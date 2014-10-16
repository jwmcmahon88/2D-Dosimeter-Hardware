#include <asf.h>
#include <inttypes.h>
#include <string.h>

#define COUNTER_PIO PIOA
#define COUNTER_PIO_ID ID_PIOA
#define COUNTER_IRQ_PRIORITY 0
#define COUNTER_STEP_PIN PIO_PA30
#define COUNTER_DIR_PIN PIO_PA31

#define COUNTER_TC TC0
#define PRIMARY_TC_CHANNEL 1
#define PRIMARY_TC_CHANNEL_ID ID_TC1
#define SECONDARY_TC_CHANNEL 2
#define SECONDARY_TC_CHANNEL_ID ID_TC2
#define TERTIARY_TC_CHANNEL 3
#define TERTIARY_TC_CHANNEL_ID ID_TC3


// The maximum offset of the head in steps
// This defines the size of the count buffers
#define HEAD_STEPS_MAX 8000

// The current relative position (in steps) of the head
volatile int32_t head_position;

// Counter buffers
volatile bool enable_count = false;
volatile uint16_t primary_count[HEAD_STEPS_MAX];
volatile uint16_t secondary_count[HEAD_STEPS_MAX];
volatile uint16_t tertiary_count[HEAD_STEPS_MAX];

void parse_gcode(const char *line, uint8_t length);

static void Trigger_Step(uint32_t id, uint32_t pin)
{
	int32_t head_step = pio_get(COUNTER_PIO, PIO_INPUT, COUNTER_DIR_PIN) ? 1 : -1;

	if (enable_count)
	{
		primary_count[head_position] += (uint16_t)tc_read_cv(COUNTER_TC, PRIMARY_TC_CHANNEL);
		secondary_count[head_position] += (uint16_t)tc_read_cv(COUNTER_TC, SECONDARY_TC_CHANNEL);
		tertiary_count[head_position] += (uint16_t)tc_read_cv(COUNTER_TC, TERTIARY_TC_CHANNEL);
		tc_sync_trigger(COUNTER_TC);
	}

	head_position += head_step;

	// Check limits and loop around the buffer if we overflow
	if (head_position < 0)
		head_position += HEAD_STEPS_MAX;
	
	if (head_position >= HEAD_STEPS_MAX)
		head_position -= HEAD_STEPS_MAX;
}

void parse_gcode(const char *line, uint8_t length)
{
	// Really hacky fake-gcode interpreter
	// This is only a prototype so it doesn't need to be robust.
	// Assumes that there is exactly one command per line.

	// Report current position
	if (!strcmp(line, "M1001"))
	{
		printf("ok\n");
		printf("%"PRId32"\n", head_position);
		return;
	}

	// Reset current position
	if (!strcmp(line, "M1002"))
	{
		head_position = 0;
		printf("ok\n");
		return;
	}
	
	// Enable counting
	if (!strcmp(line, "M1003"))
	{
		if (enable_count)
		{
			printf("error: counter is already active\n");
			return;
		}

		tc_sync_trigger(COUNTER_TC);
		enable_count = true;
		printf("ok\n");
		return;
	}
	
	// Disable counting
	if (!strcmp(line, "M1004"))
	{
		if (!enable_count)
		{
			printf("error: counter is not active\n");
			return;
		}

		enable_count = false;
		printf("ok\n");
		return;
	}
	
	// Read primary counts
	if (!strncmp(line, "M1005", 5))
	{
		if (enable_count)
		{
			printf("error: cannot read counter while it is active\n");
			return;
		}

		int32_t channel, start, end;
		if (sscanf(line, "M1005 %"SCNd32" %"SCNd32" %"SCNd32, &channel, &start, &end) != 3)
		{
			printf("error: read command requires three arguments\n");
			return;
		}

		if (channel < 0 || channel > 2)
		{
			printf("error: invalid counter\n");
			return;
		}

		if (start < 0 || start >= HEAD_STEPS_MAX || end < 0 || end >= HEAD_STEPS_MAX || start > end)
		{
			printf("error: invalid column range\n");
			return;
		}

		printf("ok\n");

		// TODO: This really should transfer in binary,
		// but text is easier to debug using a terminal
		volatile uint16_t *counts[3] = { primary_count, secondary_count, tertiary_count };
		for (int32_t i = start; i <= end; i++)
			printf("%"PRIu16" ", counts[channel][i]);
		printf("\n");

		printf("ok\n");
		return;
	}
	
	// Reset counts
	if (!strcmp(line, "M1006"))
	{
		if (enable_count)
		{
			printf("error: cannot reset counter while it is active\n");
			return;
		}

		memset((uint8_t *)primary_count, 0, 2 * HEAD_STEPS_MAX);
		memset((uint8_t *)secondary_count, 0, 2 * HEAD_STEPS_MAX);
		memset((uint8_t *)tertiary_count, 0, 2 * HEAD_STEPS_MAX);
		printf("ok\n");
		return;
	}

	printf("error: unknown command '%s'\n", line);
}

int main (void)
{
	sysclk_init();
	board_init();

	irq_initialize_vectors();
	cpu_irq_enable();
	stdio_usb_init();

	// Count primary counts in channel 0 (attached to TCLK0, PA4)
	// Count secondary counts in channel 2 (attached to TCLK1, PA28)
	// Count tertiary counts in channel 3 (attached to TCLK2, PA29)
	pmc_enable_periph_clk(PRIMARY_TC_CHANNEL_ID);
	pmc_enable_periph_clk(SECONDARY_TC_CHANNEL_ID);
	pmc_enable_periph_clk(TERTIARY_TC_CHANNEL_ID);
	pmc_enable_periph_clk(COUNTER_PIO_ID);

	pio_configure(COUNTER_PIO, PIO_TYPE_PIO_PERIPH_B, PIO_PA4 | PIO_PA28 | PIO_PA29, 0);

	tc_init(COUNTER_TC, PRIMARY_TC_CHANNEL, TC_CMR_TCCLKS_XC0); // TCLK0 -> PA4
	tc_init(COUNTER_TC, SECONDARY_TC_CHANNEL, TC_CMR_TCCLKS_XC1); // TCLK1 -> PA28
	tc_init(COUNTER_TC, TERTIARY_TC_CHANNEL, TC_CMR_TCCLKS_XC2); // TCLK2 -> PA29

	tc_start(COUNTER_TC, PRIMARY_TC_CHANNEL);
	tc_start(COUNTER_TC, SECONDARY_TC_CHANNEL);
	tc_start(COUNTER_TC, TERTIARY_TC_CHANNEL);

	// Enable step tracking
	pmc_enable_periph_clk(COUNTER_PIO_ID);
	pio_configure(COUNTER_PIO, PIO_TYPE_PIO_INPUT, COUNTER_STEP_PIN | COUNTER_DIR_PIN, 0);
	pio_handler_set(COUNTER_PIO, ID_PIOA, COUNTER_STEP_PIN, PIO_IT_RISE_EDGE, Trigger_Step);
	pio_set_input(COUNTER_PIO, COUNTER_STEP_PIN, PIO_DEGLITCH);

	NVIC_EnableIRQ((IRQn_Type)COUNTER_PIO_ID);
	pio_handler_set_priority(COUNTER_PIO, (IRQn_Type)COUNTER_PIO_ID, COUNTER_IRQ_PRIORITY);
	pio_enable_interrupt(COUNTER_PIO, COUNTER_STEP_PIN);

	// The main loop only needs to parse commands.
	// The counting and position monitoring is handled by interrupts.
	char linebuf[256];
	uint8_t buflen = 0;

	while (true)
	{
		while (udi_cdc_is_rx_ready())
		{
			// buflen will wrap to 0 on increment.  Notify the caller of the data loss
			if (buflen == 255)
			printf("WARNING: input buffer full.  Buffered data have been discarded.\r\n");

			char c = udi_cdc_getc();
			if (c == '\n' || c == '\r')
			{
				linebuf[buflen++] = '\0';
				parse_gcode(linebuf, buflen);
				buflen = 0;
			}
			else
				linebuf[buflen++] = c;
		}
	}
}
