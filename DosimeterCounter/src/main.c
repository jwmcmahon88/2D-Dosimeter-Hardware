
#include <asf.h>
#include <inttypes.h>
#include <string.h>

#define COUNTER_PIO PIOA
#define COUNTER_PIO_ID ID_PIOA
#define COUNTER_IRQ_PRIORITY 0
#define COUNTER_PRIMARY_PIN PIO_PA13
#define COUNTER_SECONDRY_PIN PIO_PA14
#define COUNTER_STEP_PIN PIO_PA4
#define COUNTER_DIR_PIN PIO_PA3

// The maximum offset of the head in steps
// This defines the size of the count buffers
#define HEAD_STEPS_MAX 8000

// The current relative position (in steps) of the head
volatile int32_t head_position;

// Counter buffers
volatile bool enable_count = false;
volatile uint16_t primary_count[HEAD_STEPS_MAX];
volatile uint16_t secondary_count[HEAD_STEPS_MAX];

void parse_gcode(const char *line, uint8_t length);

static void Trigger_Primary(uint32_t id, uint32_t pin)
{
	if (!enable_count)
		return;

	uint16_t current = primary_count[head_position];

	// Avoid counter overflow
	if (current < UINT16_MAX)
		primary_count[head_position]++;
}

static void Trigger_Secondary(uint32_t id, uint32_t pin)
{
	if (!enable_count)
		return;

	uint16_t current = secondary_count[head_position];

	// Avoid counter overflow
	if (current < UINT16_MAX)
		secondary_count[head_position]++;
}

static void Trigger_Step(uint32_t id, uint32_t pin)
{
	head_position += pio_get(COUNTER_PIO, PIO_INPUT, COUNTER_DIR_PIN) ? 1 : -1;

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
		printf("ok\r\n");
		printf("%"PRId32"\r\n", head_position);
		return;
	}

	// Reset current position
	if (!strcmp(line, "M1002"))
	{
		head_position = 0;
		printf("ok\r\n");
		return;
	}
	
	// Enable counting
	if (!strcmp(line, "M1003"))
	{
		if (enable_count)
		{
			printf("error: counter is already active\r\n");
			return;
		}

		enable_count = true;
		printf("ok\r\n");
		return;
	}
	
	// Disable counting
	if (!strcmp(line, "M1004"))
	{
		if (!enable_count)
		{
			printf("error: counter is not active\r\n");
			return;
		}

		enable_count = false;
		printf("ok\r\n");
		return;
	}
	
	// Read primary counts
	if (!strncmp(line, "M1005", 5))
	{
		if (enable_count)
		{
			printf("error: cannot read counter while it is active\r\n");
			return;
		}

		int32_t channel, start, end;
		if (sscanf(line, "M1005 %"SCNd32" %"SCNd32" %"SCNd32, &channel, &start, &end) != 3)
		{
			printf("error: read command requires three arguments\r\n");
			return;
		}

		if (channel != 0 && channel != 1)
		{
			printf("error: channel must be 0 or 1\r\n");
			return;
		}

		if (start < 0 || start >= HEAD_STEPS_MAX)
		{
			printf("error: start column must be in the range 0..%d\r\n", HEAD_STEPS_MAX);
			return;
		}
		
		if (end < 0 || end >= HEAD_STEPS_MAX)
		{
			printf("error: end column must be in the range 0..%d\r\n", HEAD_STEPS_MAX);
			return;
		}

		if (start > end)
		{
			printf("error: start column must less than or equal to end column\r\n");
			return;
		}

		printf("ok\r\n");

		// TODO: This really should transfer in binary,
		// but text is easier to debug using a terminal
		uint16_t *output = channel == 1 ? secondary_count : primary_count;
		for (int32_t i = start; i <= end; i++)
			printf("%u ", output[i]);
		printf("\r\n");

		printf("ok\r\n");
		return;
	}
	
	// Reset counts
	if (!strcmp(line, "M1006"))
	{
		if (enable_count)
		{
			printf("error: cannot reset counter while it is active\r\n");
			return;
		}

		memset((uint16_t *)primary_count, 0, 2 * HEAD_STEPS_MAX);
		memset((uint16_t *)secondary_count, 0, 2 * HEAD_STEPS_MAX);
		printf("ok\r\n");
		return;
	}
	
	printf("error: unknown command '%s'\r\n", line);
}

int main (void)
{
	sysclk_init();
	board_init();

	irq_initialize_vectors();
	cpu_irq_enable();
	stdio_usb_init();

	// Set up input pins
	pmc_enable_periph_clk(COUNTER_PIO_ID);
	pio_configure(COUNTER_PIO, PIO_TYPE_PIO_INPUT, COUNTER_PRIMARY_PIN | COUNTER_SECONDRY_PIN | COUNTER_STEP_PIN | COUNTER_DIR_PIN, 0);

	pio_handler_set(COUNTER_PIO, ID_PIOA, COUNTER_PRIMARY_PIN, PIO_DEBOUNCE | PIO_IT_RISE_EDGE, Trigger_Primary);
	pio_handler_set(COUNTER_PIO, ID_PIOA, COUNTER_SECONDRY_PIN, PIO_DEBOUNCE | PIO_IT_RISE_EDGE, Trigger_Secondary);
	pio_handler_set(COUNTER_PIO, ID_PIOA, COUNTER_STEP_PIN, PIO_DEBOUNCE | PIO_IT_RISE_EDGE, Trigger_Step);
	NVIC_EnableIRQ((IRQn_Type)COUNTER_PIO_ID);
	pio_handler_set_priority(COUNTER_PIO, (IRQn_Type)COUNTER_PIO_ID, COUNTER_IRQ_PRIORITY);
	pio_enable_interrupt(COUNTER_PIO, COUNTER_PRIMARY_PIN | COUNTER_SECONDRY_PIN | COUNTER_STEP_PIN);

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
			if (c == '\r' || c == '\n')
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