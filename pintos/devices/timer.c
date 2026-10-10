#include "devices/timer.h"
#include <debug.h>
#include <inttypes.h>
#include <round.h>
#include <stdio.h>
#include "threads/interrupt.h"
#include "threads/io.h"
#include "threads/synch.h"
#include "threads/thread.h"

/* See [8254] for hardware details of the 8254 timer chip. */

#if TIMER_FREQ < 19
#error 8254 timer requires TIMER_FREQ >= 19
#endif
#if TIMER_FREQ > 1000
#error TIMER_FREQ <= 1000 recommended
#endif

/* Number of timer ticks since OS booted. */
static int64_t ticks;

/* Number of loops per timer tick.
   Initialized by timer_calibrate(). */
static unsigned loops_per_tick;

static intr_handler_func timer_interrupt;
static bool too_many_loops(unsigned loops);
static void busy_wait(int64_t loops);
static void real_time_sleep(int64_t num, int32_t denom);

/* Sets up the 8254 Programmable Interval Timer (PIT) to
   interrupt PIT_FREQ times per second, and registers the
   corresponding interrupt. */
void timer_init(void)
{
	/* 8254 input frequency divided by TIMER_FREQ, rounded to
	   nearest. */
	uint16_t count = (1193180 + TIMER_FREQ / 2) / TIMER_FREQ; // PIT 는 1초에 약 1,193180 번 쿨럭이 발생함, 몇번의 쿨럭마다 인터럽트 발생 시킬지

	// outb(포트번호,전달할 값))
	outb(0x43, 0x34); /* CW: counter 0, LSB then MSB, mode 2, binary. */
	outb(0x40, count & 0xff);
	outb(0x40, count >> 8);

	intr_register_ext(0x20, timer_interrupt, "8254 Timer"); // PIT 타이머에서 인터럽트 발생시 timer_interrupt()함수 실행
}

/* Calibrates loops_per_tick, used to implement brief delays. cpu 마다 속도가 다르므로 타이머 교정*/
void timer_calibrate(void)
{
	unsigned high_bit, test_bit;

	ASSERT(intr_get_level() == INTR_ON);
	printf("Calibrating timer...  ");

	/* Approximate loops_per_tick as the largest power-of-two
	   still less than one timer tick. */
	loops_per_tick = 1u << 10;
	while (!too_many_loops(loops_per_tick << 1))
	{
		loops_per_tick <<= 1;
		ASSERT(loops_per_tick != 0);
	}

	/* Refine the next 8 bits of loops_per_tick. */
	high_bit = loops_per_tick;
	for (test_bit = high_bit >> 1; test_bit != high_bit >> 10; test_bit >>= 1)
		if (!too_many_loops(high_bit | test_bit))
			loops_per_tick |= test_bit;

	printf("%'" PRIu64 " loops/s.\n", (uint64_t)loops_per_tick * TIMER_FREQ);
}

/* Returns the number of timer ticks since the OS booted. */ // Pintos가 시작된 이후 현재까지 몇 틱이 지났는지 확인
// ticks 는 전역변수임
// 그냥 현재 시간 확인하기임, 인터럽트 끄는거는 tick 값을 읽는 짧은 구간을 보호하기 위해서임
int64_t timer_ticks(void)
{
	enum intr_level old_level = intr_disable(); // 인터럽트 끄기 + 끄기전 상황 반환하기(원래 켜져있었나 꺼져있었나)
	int64_t t = ticks;
	intr_set_level(old_level);
	barrier(); // 컴파일러 최적화 제한. 알람 단계에서는 크게 신경안써도 됨
	return t;
}

/* Returns the number of timer ticks elapsed since THEN, which
   should be a value once returned by timer_ticks(). */
int64_t timer_elapsed(int64_t then)
{
	return timer_ticks() - then;
}

/* Suspends execution for approximately TICKS timer ticks. */
// ticks 경과 만큼 스레드의 실행을 중단한다. 지정된 시간만큼 기다렸으면 ready queue 에 넣어주면 된다
// 단위는 틱(Tick), 1초당 타이머 틱의 수는 TIMER_FREQ 로 결정 된다. devices/timer.h 에 정의됨
// 기본값은 100
// 틱 만큼 재우겠다!
void timer_sleep(int64_t ticks)
{
	ASSERT(intr_get_level() == INTR_ON);
	if (ticks <= 0)
		return;
	enum intr_level old_level = intr_disable();

	int64_t start = timer_ticks();
	int64_t wakeup_tick = start + ticks;
	thread_sleep(wakeup_tick);
	intr_set_level(old_level);
}

/* Suspends execution for approximately MS milliseconds. */
void timer_msleep(int64_t ms)
{
	real_time_sleep(ms, 1000);
}

/* Suspends execution for approximately US microseconds. */
void timer_usleep(int64_t us)
{
	real_time_sleep(us, 1000 * 1000);
}

/* Suspends execution for approximately NS nanoseconds. */
void timer_nsleep(int64_t ns)
{
	real_time_sleep(ns, 1000 * 1000 * 1000);
}

/* Prints timer statistics. */
void timer_print_stats(void)
{
	printf("Timer: %" PRId64 " ticks\n", timer_ticks());
}

/* Timer interrupt handler. */
static void timer_interrupt(struct intr_frame *args UNUSED)
{
	ticks++;
	thread_tick();
	thread_awake(ticks);
}

/* Returns true if LOOPS iterations waits for more than one timer
   tick, otherwise false. */
static bool too_many_loops(unsigned loops)
{
	/* Wait for a timer tick. */
	int64_t start = ticks;
	while (ticks == start)
		barrier();

	/* Run LOOPS loops. */
	start = ticks;
	busy_wait(loops);

	/* If the tick count changed, we iterated too long. */
	barrier();
	return start != ticks;
}

/* Iterates through a simple loop LOOPS times, for implementing
   brief delays.

   Marked NO_INLINE because code alignment can significantly
   affect timings, so that if this function was inlined
   differently in different places the results would be difficult
   to predict. */
static void NO_INLINE busy_wait(int64_t loops)
{
	while (loops-- > 0)
		barrier();
}

/* Sleep for approximately NUM/DENOM seconds. */
static void real_time_sleep(int64_t num, int32_t denom)
{
	/* Convert NUM/DENOM seconds into timer ticks, rounding down.

	   (NUM / DENOM) s
	   ---------------------- = NUM * TIMER_FREQ / DENOM ticks.
	   1 s / TIMER_FREQ ticks
	   */
	int64_t ticks = num * TIMER_FREQ / denom;

	ASSERT(intr_get_level() == INTR_ON);
	if (ticks > 0)
	{
		/* We're waiting for at least one full timer tick.  Use
		   timer_sleep() because it will yield the CPU to other
		   processes. */
		timer_sleep(ticks);
	}
	else
	{
		/* Otherwise, use a busy-wait loop for more accurate
		   sub-tick timing.  We scale the numerator and denominator
		   down by 1000 to avoid the possibility of overflow. */
		ASSERT(denom % 1000 == 0);
		busy_wait(loops_per_tick * num / 1000 * TIMER_FREQ / (denom / 1000));
	}
}
