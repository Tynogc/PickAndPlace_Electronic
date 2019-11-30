#include "ws2812.h"

void ws2812_init(ws2812_t *data) {
	// Verwende Timer 3 - Kanal 3 DMA1-Ch3 - Ausgang PB0

	memset(data->dma_buf, 0, data->dma_len);
	memset((void*) data->led_buf, 0, data->count * sizeof(data->led_buf));

	gpio_mode_setup(data->port, GPIO_MODE_AF, GPIO_PUPD_NONE, data->pin);
	gpio_set_af(data->port, data->af_number, data->pin);
	gpio_set_output_options(data->port, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, data->pin);

	rcc_periph_reset_pulse(data->timer_rcc);

	timer_set_mode(data->timernumber, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_continuous_mode(data->timernumber);
	timer_set_period(data->timernumber, WSP);

	timer_disable_oc_output(data->timernumber, data->timerchannel);

	timer_set_oc_mode(data->timernumber, data->timerchannel, TIM_OCM_PWM1);
	timer_disable_oc_clear(data->timernumber, data->timerchannel);
	timer_enable_oc_preload(data->timernumber, data->timerchannel);
	timer_set_oc_value(data->timernumber, data->timerchannel, 0);
	timer_set_oc_polarity_high(data->timernumber, data->timerchannel);
	timer_enable_oc_output(data->timernumber, data->timerchannel);

	timer_enable_preload(data->timernumber);
	timer_set_counter(data->timernumber, 0);

	timer_enable_counter(data->timernumber);

	timer_enable_irq(data->timernumber, TIM_DIER_UDE); // Update DMA request
	timer_set_dma_on_update_event(data->timernumber);
}

void ws2812_dma_init(ws2812_t *data) {

	dma_disable_channel(data->dmanumber, data->dmachannel);

	nvic_set_priority(data->dmainterrupt, 0);
	nvic_enable_irq(data->dmainterrupt);

	// Reset DMA channels
	dma_channel_reset(data->dmanumber, data->dmachannel);

	// Set up tx dma
	dma_set_peripheral_address(data->dmanumber, data->dmachannel, (uint32_t) & TIM3_CCR3);
	dma_set_memory_address(data->dmanumber, data->dmachannel, (uint32_t) data->dma_buf);
	dma_set_number_of_data(data->dmanumber, data->dmachannel, data->dma_len);
	dma_set_read_from_memory(data->dmanumber, data->dmachannel);
	dma_enable_memory_increment_mode(data->dmanumber, data->dmachannel);
	dma_set_peripheral_size(data->dmanumber, data->dmachannel, DMA_CCR_PSIZE_16BIT);
	dma_set_memory_size(data->dmanumber, data->dmachannel, DMA_CCR_MSIZE_8BIT);
	dma_set_priority(data->dmanumber, data->dmachannel, DMA_CCR_PL_HIGH);

	dma_enable_circular_mode(data->dmanumber, data->dmachannel);
	dma_enable_transfer_complete_interrupt(data->dmanumber, data->dmachannel);
	dma_enable_half_transfer_interrupt(data->dmanumber, data->dmachannel);
	dma_enable_transfer_error_interrupt(data->dmanumber, data->dmachannel);
	dma_enable_channel(data->dmanumber, data->dmachannel);
}

void ws2812_generate_data(ws2812_t *data, uint8_t *dma_data_startpointer) {
	for (int i = 0; i < (data->dma_len >> 1);) {
		data->current_led = data->current_led % (data->count + 5);
		if (data->current_led < data->count) {
			uint32_t v = data->led_buf[data->current_led];
			for (int j = 0; j < 24; j++) { // schreibe untere 24 Bit jedes 32-Bit Wertes
				dma_data_startpointer[i++] = (v & 0x800000) ? (uint8_t) WS1 : (uint8_t) WS0; // Timerwert für 1 oder 0 schreiben
				v <<= 1; // Daten der aktuellen LED eins nach Links schieben
			}
		} else {
			for (int j = 0; j < 24; j++) { // erzeuge 24 weitere Bit als Whitespace (WS2812-Latch)
				data->dma_buf[i++] = 0;
			}
		}
		data->current_led++;
	}
}



