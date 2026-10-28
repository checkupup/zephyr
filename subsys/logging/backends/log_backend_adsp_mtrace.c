/*
 * Copyright (c) 2019,2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log_backend_adsp_mtrace.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_core.h>
#include <zephyr/logging/log_output.h>
#include <zephyr/logging/log_backend_std.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <soc.h>

#include <adsp_memory.h>
#include <adsp_debug_window.h>

/*
 * A lock is needed as log_process() and log_panic() have no internal locks
 * to prevent concurrency. Meaning if log_process is called after
 * log_panic was called previously, log_process may happen from another
 * CPU and calling context than the log processing thread running in
 * the background. On an SMP system this is a race.
 *
 * This caused a race on the output trace such that the logging output
 * was garbled and useless.
 */
static struct k_spinlock mtrace_lock;

static uint32_t log_format_current = CONFIG_LOG_BACKEND_ADSP_MTRACE_OUTPUT_DEFAULT;

static adsp_mtrace_log_hook_t mtrace_hook;

static bool mtrace_active;

static bool mtrace_panic_mode;

/*
 * SRAM window for debug info is organized to equal size slots.
 * The descriptors on first page describe the layout of slots.
 * One type of debug info slot is ADSP_DBG_WIN_SLOT_DEBUG_LOG.
 * These slots (if defined) can be used for mtrace output.
 *
 * The log buffer slots have the following layout:
 *
 * u32 host_read_ptr;
 * u32 dsp_write_ptr;
 * u8 buffer[];
 *
 * The two pointers are offsets within the buffer. Buffer is empty
 * when the two pointers are equal, and full when host read pointer
 * is one ahead of the DSP writer pointer.
 */

#define MTRACE_LOG_BUF_SIZE		(ADSP_DW_SLOT_SIZE - 2 * sizeof(uint32_t))

#define MTRACE_LOGGING_SLOT_TYPE(n)	(ADSP_DW_SLOT_DEBUG_LOG | ((n) & ADSP_DW_SLOT_CORE_MASK))

#define MTRACE_CORE		0

struct adsp_debug_slot {
	uint32_t host_ptr;
	uint32_t dsp_ptr;
	uint8_t data[ADSP_DW_SLOT_SIZE - sizeof(uint32_t) * 2];
} __packed;

#ifdef CONFIG_INTEL_ADSP_DEBUG_SLOT_MANAGER
#ifdef CONFIG_LOG_MULTI_SLOT
static struct adsp_debug_slot *slots[CONFIG_MP_MAX_NUM_CPUS];
#else
static struct adsp_debug_slot *slots[1];
#endif
#endif

static void mtrace_init(void)
{
#ifdef CONFIG_INTEL_ADSP_DEBUG_SLOT_MANAGER
#ifdef CONFIG_LOG_MULTI_SLOT
	for (unsigned int core = 0; core < CONFIG_MP_MAX_NUM_CPUS; core++) {
		if (!slots[core]) {
			struct adsp_dw_desc desc = {
				.type = MTRACE_LOGGING_SLOT_TYPE(core),
			};

			slots[core] = adsp_dw_request_slot(&desc, NULL);
			if (slots[core]) {
				slots[core]->host_ptr = 0;
				slots[core]->dsp_ptr = 0;
			}
		}
	}
#else
	struct adsp_dw_desc slot_desc = { .type = MTRACE_LOGGING_SLOT_TYPE(MTRACE_CORE), };

	if (slots[0]) {
		return;
	}

	slots[0] = adsp_dw_request_slot(&slot_desc, NULL);
	if (slots[0]) {
		slots[0]->host_ptr = 0;
		slots[0]->dsp_ptr = 0;
	}
#endif
#else
	struct adsp_debug_slot *slot = (struct adsp_debug_slot *)
		ADSP_DW->slots[ADSP_DW_SLOT_NUM_MTRACE];

	if (ADSP_DW->descs[ADSP_DW_SLOT_NUM_MTRACE].type == MTRACE_LOGGING_SLOT_TYPE(MTRACE_CORE)) {
		return;
	}

	ADSP_DW->descs[ADSP_DW_SLOT_NUM_MTRACE].type = MTRACE_LOGGING_SLOT_TYPE(MTRACE_CORE);
	slot->host_ptr = 0;
	slot->dsp_ptr = 0;
#endif
}

static size_t mtrace_out(int8_t *str, size_t len, size_t *space_left,
			 unsigned int core)
{
#ifdef CONFIG_INTEL_ADSP_DEBUG_SLOT_MANAGER
	struct adsp_debug_slot *slot = slots[core];

	/* Debug slot is not allocated */
	if (!slot) {
		return 0;
	}
#else
	struct adsp_debug_slot *slot = (struct adsp_debug_slot *)
		ADSP_DW->slots[ADSP_DW_SLOT_NUM_MTRACE];
#endif

	uint8_t *data = slot->data;
	uint32_t r = slot->host_ptr;
	uint32_t w = slot->dsp_ptr;
	size_t avail, out;

	if (w > r) {
		avail = MTRACE_LOG_BUF_SIZE - w + r - 1;
	} else if (w == r) {
		avail = MTRACE_LOG_BUF_SIZE - 1;
	} else {
		avail = r - w - 1;
	}

	if (len == 0) {
		out = 0;
		goto out;
	}

	/* data that does not fit is dropped */
	out = MIN(avail, len);

	if (w + out >= MTRACE_LOG_BUF_SIZE) {
		size_t tail = MTRACE_LOG_BUF_SIZE - w;
		size_t head = out - tail;

		memcpy(data + w, str, tail);
		memcpy(data, str + tail, head);
		w = head;
	} else {
		memcpy(data + w, str, out);
		w += out;
	}

	slot->dsp_ptr = w;

out:
	if (space_left) {
		*space_left = avail > len ? avail - len : 0;
	}

	return out;
}

#if CONFIG_MTRACE_LOG_CACHE_BUF_SIZE > ADSP_DW_SLOT_SIZE
/*
 * The cache buffer absorbs bursts the debug window slot cannot take because
 * the host has not consumed it yet. Data moves from the cache into the slot
 * only once the host has made room, which it does independently of DSP log
 * activity. Draining solely from char_out() would leave the tail of a burst in
 * the cache forever, hence the periodic drain below.
 */
#ifdef CONFIG_LOG_MULTI_SLOT
#define MTRACE_CACHE_COUNT	CONFIG_MP_MAX_NUM_CPUS
#else
#define MTRACE_CACHE_COUNT	1
#endif

struct mtrace_cache {
	struct k_spinlock lock;
	size_t head;
	size_t len;
	uint32_t dropped_bytes;
	bool discarding;
	uint8_t buf[CONFIG_MTRACE_LOG_CACHE_BUF_SIZE];
};

static struct mtrace_cache mtrace_caches[MTRACE_CACHE_COUNT];

/*
 * Consecutive periodic drain rounds that moved nothing while data was pending.
 * A host can stop consuming the slot at any time; retrying at the configured
 * period forever would keep a core awake for nothing, so the period backs off
 * from this counter. Any drain that moves data resets it. Read and written from
 * the drain work and from char_out(), hence atomic.
 */
static atomic_t mtrace_drain_stalled;

#define MTRACE_DRAIN_STALL_ROUNDS	8U
#define MTRACE_DRAIN_BACKOFF_MAX_MS	1000U
/*
 * The period doubles once per MTRACE_DRAIN_STALL_ROUNDS stalled rounds and is
 * capped at MTRACE_DRAIN_BACKOFF_MAX_MS, so this many rounds already sits at the
 * cap; the counter saturates there instead of growing without bound.
 */
#define MTRACE_DRAIN_STALL_MAX		(MTRACE_DRAIN_STALL_ROUNDS * 10U)

static bool mtrace_drain_armed;

static void mtrace_drain_handler(struct k_work *work);

static K_WORK_DELAYABLE_DEFINE(mtrace_drain_work, mtrace_drain_handler);

static struct mtrace_cache *mtrace_cache_get(unsigned int core)
{
	if (core >= MTRACE_CACHE_COUNT) {
		core = 0;
	}

	return &mtrace_caches[core];
}

static void mtrace_cache_append(struct mtrace_cache *cache, const uint8_t *src,
				size_t len)
{
	size_t w = (cache->head + cache->len) % CONFIG_MTRACE_LOG_CACHE_BUF_SIZE;
	size_t first = MIN(len, CONFIG_MTRACE_LOG_CACHE_BUF_SIZE - w);

	memcpy(&cache->buf[w], src, first);

	if (first < len) {
		memcpy(&cache->buf[0], src + first, len - first);
	}

	cache->len += len;
}

/*
 * Runs with the cache lock held. Drops a trailing partial line so that the
 * cache only ever holds whole lines. The output function is called with chunks
 * that follow the log_output buffer boundaries, so a line longer than that
 * buffer arrives in several pieces and a piece may be left dangling when the
 * rest of the line has to be dropped.
 */
static void mtrace_cache_trim_partial(struct mtrace_cache *cache)
{
	while (cache->len > 0) {
		size_t idx = (cache->head + cache->len - 1) %
			     CONFIG_MTRACE_LOG_CACHE_BUF_SIZE;

		if (cache->buf[idx] == '\n') {
			return;
		}

		cache->len--;
		cache->dropped_bytes++;
	}
}

/*
 * Runs with the cache lock held. A partial log line is never left in the cache:
 * when the remaining room cannot hold a whole line, the rest of that line is
 * dropped and the next line boundary is used to resynchronise. The number of
 * dropped bytes is accumulated so it can be reported later.
 */
static void mtrace_cache_fill(struct mtrace_cache *cache, const uint8_t *data,
			      size_t length)
{
	size_t offset = 0;

	if (cache->discarding) {
		const uint8_t *nl = memchr(data, '\n', length);

		if (nl == NULL) {
			cache->dropped_bytes += (uint32_t)length;
			return;
		}

		offset = (size_t)(nl - data) + 1;
		cache->dropped_bytes += (uint32_t)offset;
		cache->discarding = false;
	}

	while (offset < length) {
		size_t room = CONFIG_MTRACE_LOG_CACHE_BUF_SIZE - cache->len;
		size_t remain = length - offset;
		const uint8_t *nl;
		size_t keep = 0;

		if (remain <= room) {
			mtrace_cache_append(cache, data + offset, remain);
			break;
		}

		/* Keep the longest prefix ending on a line boundary that fits. */
		for (size_t i = room; i > 0; i--) {
			if (data[offset + i - 1] == '\n') {
				keep = i;
				break;
			}
		}

		if (keep > 0) {
			mtrace_cache_append(cache, data + offset, keep);
			offset += keep;
			continue;
		}

		/*
		 * No whole line fits: drop the remainder of this line rather than
		 * leaving a truncated one behind, then resynchronise.
		 */
		mtrace_cache_trim_partial(cache);

		nl = memchr(data + offset, '\n', remain);

		if (nl == NULL) {
			cache->dropped_bytes += (uint32_t)remain;
			cache->discarding = true;
			break;
		}

		keep = (size_t)(nl - (data + offset)) + 1;
		cache->dropped_bytes += (uint32_t)keep;
		offset += keep;
	}
}

/* Last byte held by the cache, or '\n' when the cache is empty. */
static uint8_t mtrace_cache_tail(struct mtrace_cache *cache)
{
	size_t idx;

	if (cache->len == 0) {
		return '\n';
	}

	idx = (cache->head + cache->len - 1) % CONFIG_MTRACE_LOG_CACHE_BUF_SIZE;

	return cache->buf[idx];
}

/* Runs with the cache lock held. Emits a notice for previously dropped data. */
static void mtrace_cache_notice(struct mtrace_cache *cache)
{
	char msg[64];
	size_t room;
	int len;

	if (cache->dropped_bytes == 0) {
		return;
	}

	/* Only ever start on a line boundary. */
	if (mtrace_cache_tail(cache) != '\n') {
		return;
	}

	len = snprintk(msg, sizeof(msg), "--- %u bytes dropped ---\r\n",
		       cache->dropped_bytes);
	if (len <= 0) {
		return;
	}

	room = (size_t)CONFIG_MTRACE_LOG_CACHE_BUF_SIZE - cache->len;
	if ((size_t)len > room) {
		/* No room yet, retry once the host has consumed the cache. */
		return;
	}

	mtrace_cache_append(cache, (const uint8_t *)msg, (size_t)len);
	cache->dropped_bytes = 0;
}

/* Runs with the cache lock held. Moves cached data into the slot. */
static size_t mtrace_cache_drain(struct mtrace_cache *cache, unsigned int core,
				 size_t *space_left)
{
	size_t chunk;
	size_t out;

	if (cache->len == 0) {
		/*
		 * Nothing to move, but the caller still needs the real free space
		 * of the slot for the notification watermark. A zero length write
		 * only reports the free space and stores nothing.
		 */
		(void)mtrace_out(NULL, 0, space_left, core);
		return 0;
	}

	chunk = MIN(cache->len, CONFIG_MTRACE_LOG_CACHE_BUF_SIZE - cache->head);

	out = mtrace_out((int8_t *)cache->buf + cache->head, chunk, space_left, core);

	if (*space_left > 0 && cache->len > chunk) {
		out += mtrace_out((int8_t *)cache->buf,
				  MIN(*space_left, cache->len - chunk), space_left,
				  core);
	}

	cache->head = (cache->head + out) % CONFIG_MTRACE_LOG_CACHE_BUF_SIZE;
	cache->len -= out;

	return out;
}

/*
 * Next retry delay. The configured period is used while the host keeps making
 * room, and doubled for every MTRACE_DRAIN_STALL_ROUNDS consecutive rounds
 * that moved nothing, up to MTRACE_DRAIN_BACKOFF_MAX_MS. The retries never stop
 * entirely, so data still reaches the host once it starts reading again.
 */
static k_timeout_t mtrace_drain_delay(void)
{
	unsigned int period = CONFIG_MTRACE_LOG_CACHE_DRAIN_PERIOD_MS;
	unsigned int shift = (unsigned int)atomic_get(&mtrace_drain_stalled) /
			     MTRACE_DRAIN_STALL_ROUNDS;

	while (shift > 0U) {
		period = MIN(period * 2U, MTRACE_DRAIN_BACKOFF_MAX_MS);
		shift--;
	}

	return K_MSEC(period);
}

static void mtrace_drain_schedule(void)
{
	if (!mtrace_drain_armed || k_is_pre_kernel()) {
		return;
	}

	if (atomic_get(&mtrace_drain_stalled) == 0) {
		/* Draining: keep the retry at the configured period. */
		(void)k_work_reschedule(&mtrace_drain_work,
					K_MSEC(CONFIG_MTRACE_LOG_CACHE_DRAIN_PERIOD_MS));
	} else {
		/*
		 * Stalled: leave a pending retry on its backed off deadline.
		 * Rescheduling here would let a steady stream of log writes
		 * postpone the retry indefinitely.
		 */
		(void)k_work_schedule(&mtrace_drain_work, mtrace_drain_delay());
	}
}

/*
 * Feeds one drain round into the backoff counter: a round that moved data
 * clears it, a round that could not move pending data counts towards the next
 * backoff step. Counting rounds rather than bytes means an empty cache is not a
 * stall, and the counter is backend wide, so a core starved of slot space while
 * another one makes progress does not back off.
 */
static void mtrace_drain_account(size_t moved, bool pending)
{
	if (moved > 0) {
		atomic_set(&mtrace_drain_stalled, 0);
	} else if (pending && ((unsigned int)atomic_get(&mtrace_drain_stalled) <
			       MTRACE_DRAIN_STALL_MAX)) {
		atomic_inc(&mtrace_drain_stalled);
	}
}

/*
 * Moves whatever the caches hold into the slots. Scheduling is left to the
 * caller, because the panic path must not reschedule work. Returns true while
 * data is still cached.
 */
static bool mtrace_drain(void)
{
	size_t moved = 0;
	bool pending = false;

	for (unsigned int core = 0; core < MTRACE_CACHE_COUNT; core++) {
		struct mtrace_cache *cache = &mtrace_caches[core];
		k_spinlock_key_t key;
		size_t space_left = 0;
		size_t written;

		key = k_spin_lock(&cache->lock);
		written = mtrace_cache_drain(cache, core, &space_left);

		/* Report earlier losses as soon as the cache has room for it. */
		if (cache->dropped_bytes > 0) {
			mtrace_cache_notice(cache);
			written += mtrace_cache_drain(cache, core, &space_left);
		}

		if (cache->len > 0) {
			pending = true;
		}

		k_spin_unlock(&cache->lock, key);

		if (written > 0 && mtrace_hook != NULL) {
			mtrace_hook(written, space_left);
		}

		moved += written;
	}

	mtrace_drain_account(moved, pending);

	return pending;
}

static void mtrace_drain_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (mtrace_drain()) {
		mtrace_drain_schedule();
	}
}
#endif /* CONFIG_MTRACE_LOG_CACHE_BUF_SIZE > ADSP_DW_SLOT_SIZE */

static int char_out(uint8_t *data, size_t length, void *ctx)
{
	size_t space_left = 0;
	size_t out;
#ifdef CONFIG_LOG_MULTI_SLOT
	unsigned int core = (unsigned int)(uintptr_t)ctx;
#else
	unsigned int core = 0;
#endif

	/*
	 * we handle the data even if mtrace notifier is not
	 * active. this ensures we can capture early boot messages.
	 */
#if CONFIG_MTRACE_LOG_CACHE_BUF_SIZE > ADSP_DW_SLOT_SIZE
	struct mtrace_cache *cache = mtrace_cache_get(core);
	k_spinlock_key_t key = k_spin_lock(&cache->lock);
	bool pending;

	/* Report earlier losses before the data that follows them. */
	mtrace_cache_notice(cache);

	mtrace_cache_fill(cache, data, length);

	out = mtrace_cache_drain(cache, core, &space_left);
	pending = (cache->len > 0);

	k_spin_unlock(&cache->lock, key);

	/*
	 * This write moved data, so the periodic drain no longer needs to back
	 * off; rearm it at the configured period.
	 */
	mtrace_drain_account(out, pending);

	if (pending) {
		mtrace_drain_schedule();
	}
#else
	out = mtrace_out(data, length, &space_left, core);
#endif

	if (mtrace_active && mtrace_hook) {

		/* if we are in panic mode, need to flush out asap */
		if (unlikely(mtrace_panic_mode)) {
			space_left = 0;
		}

		mtrace_hook(out, space_left);
	}

	return length;
}

/**
 * 80 bytes seems to catch most sensibly sized log message lines
 * in one go letting the trace out call output whole complete
 * lines. This avoids the overhead of a spin lock in the trace_out
 * more often as well as avoiding entwined characters from printk if
 * LOG_PRINTK=n.
 */
#define LOG_BUF_SIZE 80
static uint8_t log_buf[LOG_BUF_SIZE];

LOG_OUTPUT_DEFINE(log_output_adsp_mtrace, char_out, log_buf, sizeof(log_buf));

static uint32_t format_flags(void)
{
	uint32_t flags = LOG_OUTPUT_FLAG_LEVEL | LOG_OUTPUT_FLAG_TIMESTAMP;

	if (IS_ENABLED(CONFIG_LOG_BACKEND_FORMAT_TIMESTAMP)) {
		flags |= LOG_OUTPUT_FLAG_FORMAT_TIMESTAMP;
	}

	if (IS_ENABLED(CONFIG_LOG_CORE_ID_PREFIX)) {
		flags |= LOG_OUTPUT_FLAG_CORE;
	}

	if (IS_ENABLED(CONFIG_LOG_THREAD_ID_PREFIX)) {
		flags |= LOG_OUTPUT_FLAG_THREAD;
	}

	return flags;
}

static void panic(struct log_backend const *const backend)
{
	mtrace_panic_mode = true;

#if CONFIG_MTRACE_LOG_CACHE_BUF_SIZE > ADSP_DW_SLOT_SIZE
	/* Push whatever is cached to the slot while the host can still read it. */
	(void)mtrace_drain();
#endif
}

static void dropped(const struct log_backend *const backend,
		    uint32_t cnt)
{
	log_output_dropped_process(&log_output_adsp_mtrace, cnt);
}

static void process(const struct log_backend *const backend,
		    union log_msg_generic *msg)
{
	log_format_func_t log_output_func = log_format_func_t_get(log_format_current);

	k_spinlock_key_t key = k_spin_lock(&mtrace_lock);

#ifdef CONFIG_LOG_MULTI_SLOT
	log_output_ctx_set(&log_output_adsp_mtrace,
			   (void *)(uintptr_t)log_msg_get_core_id(&msg->log));
#endif
	log_output_func(&log_output_adsp_mtrace, &msg->log, format_flags());

	k_spin_unlock(&mtrace_lock, key);
}

static int format_set(const struct log_backend *const backend, uint32_t log_type)
{
	log_format_current = log_type;
	return 0;
}

/**
 * Lazily initialized, while the DMA may not be setup we continue
 * to buffer log messages untilt he buffer is full.
 */
static void init(const struct log_backend *const backend)
{
	ARG_UNUSED(backend);

	mtrace_init();

#if CONFIG_MTRACE_LOG_CACHE_BUF_SIZE > ADSP_DW_SLOT_SIZE
	/* The drain work is statically defined; only enable scheduling now that
	 * the kernel is up and the slot is known.
	 */
	mtrace_drain_armed = true;
#endif
}

const struct log_backend_api log_backend_adsp_mtrace_api = {
	.process = process,
	.dropped = IS_ENABLED(CONFIG_LOG_MODE_IMMEDIATE) ? NULL : dropped,
	.panic = panic,
	.format_set = format_set,
	.init = init,
};

LOG_BACKEND_DEFINE(log_backend_adsp_mtrace, log_backend_adsp_mtrace_api, true);

void adsp_mtrace_log_init(adsp_mtrace_log_hook_t hook)
{
	mtrace_init();

#if CONFIG_MTRACE_LOG_CACHE_BUF_SIZE > ADSP_DW_SLOT_SIZE
	if (hook == NULL) {
		/* Nothing will drain the cache once the host stops reading it. */
		mtrace_drain_armed = false;
		(void)k_work_cancel_delayable(&mtrace_drain_work);
	} else {
		/* A new host may read the slot again, so restart at the period. */
		atomic_set(&mtrace_drain_stalled, 0);
		mtrace_drain_armed = true;
	}
#endif

	mtrace_hook = hook;
	mtrace_active = !!hook;
}

const struct log_backend *log_backend_adsp_mtrace_get(void)
{
	return &log_backend_adsp_mtrace;
}
