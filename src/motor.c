#include "motor.h"

static __inline__ float __attribute__((const)) reciprocal(float x);

void motor_buffer_init(fahrpfad_buf_t *data) {
	data->_read = 0;
	data->_write = 0;
}

void motor_buffer_write(fahrpfad_buf_t *data, fahrpfad_t *eingabe) {

	if (motor_buffer_full(data)) {
		return;
	}
	uint16_t next = ((data->_write + 1) & RINGBUFFER_MASK); // Nummer mit Überlauf...

	data->_data[data->_write] = *eingabe;
	data->_write = next;

}

void motor_buffer_read(fahrpfad_buf_t *data, fahrpfad_t *ausgabe) {
	if (motor_buffer_empty(data)) {
		return;
	}

	*ausgabe = data->_data[data->_read];

	data->_read = (data->_read + 1) & RINGBUFFER_MASK;

}

void motor_buffer_read_no_change(fahrpfad_buf_t *data, fahrpfad_t *ausgabe) {
	*ausgabe = data->_data[data->_read];
}

inline bool motor_buffer_full(fahrpfad_buf_t *data) {
	uint16_t next = ((data->_write + 1) & RINGBUFFER_MASK);

	if (data->_read == next) {
		return true; // voll
	}
	return false;
}

inline bool motor_buffer_empty(fahrpfad_buf_t *data) {
	if (data->_read == data->_write) { // LEER
		return true;
	}
	return false;
}

void stepper_gpio_init(motor_t *data) {

	gpio_mode_setup(data->anschluss->direction_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, data->anschluss->direction);
	gpio_mode_setup(data->anschluss->enable_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, data->anschluss->enable);
	gpio_mode_setup(data->anschluss->sleep_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, data->anschluss->sleep);
	gpio_mode_setup(data->anschluss->reset_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, data->anschluss->reset);
	gpio_mode_setup(data->anschluss->ms1_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, data->anschluss->ms1);
	gpio_mode_setup(data->anschluss->ms2_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, data->anschluss->ms2);
	gpio_mode_setup(data->anschluss->ms3_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, data->anschluss->ms3);

	gpio_clear(data->anschluss->reset_port, data->anschluss->enable); // Endstufe aus.

	gpio_clear(data->anschluss->ms1_port, data->anschluss->ms1);
	gpio_clear(data->anschluss->ms2_port, data->anschluss->ms2);
	gpio_clear(data->anschluss->ms3_port, data->anschluss->ms3);

	gpio_set(data->anschluss->reset_port, data->anschluss->reset);
	gpio_clear(data->anschluss->enable_port, data->anschluss->enable);
	gpio_set(data->anschluss->sleep_port, data->anschluss->sleep);

	// Setze GPIOs für MS-Pins am Motortreiber (MS-Level)
	switch (data->config->microstepping) {
	case HALF:
		gpio_set(data->anschluss->ms1_port, data->anschluss->ms1);
		break;
	case QUARTER:
		gpio_set(data->anschluss->ms2_port, data->anschluss->ms2);
		break;
	case EIGHT:
		gpio_set(data->anschluss->ms1_port, data->anschluss->ms1);
		gpio_set(data->anschluss->ms2_port, data->anschluss->ms2);
		break;
	case SIXTEENTH:
		gpio_set(data->anschluss->ms1_port, data->anschluss->ms1);
		gpio_set(data->anschluss->ms2_port, data->anschluss->ms2);
		gpio_set(data->anschluss->ms3_port, data->anschluss->ms3);
		break;
	default:
		// microsteppinglevel unkonfiguriert steht auf 0. nach dem INIT muss hier eine natürliche Zahl stehen.
		// daher wird der Wert nach dem Init auf "1" gesetzt (kein Microstepping aktiv).
		data->config->microstepping = 1;
	}

	// Motortreiber aus RESET-State holen
	gpio_set(data->anschluss->reset_port, data->anschluss->reset);
}

inline void stepper_set_direction(motor_t *data, rotation_t richtung) {

	// Richtung (0=CW) 	| reversed | GPIO
	// 0 				| 0 		= 0
	// 0 				| 1 		= 1
	// 1 				| 0 		= 1
	// 1 				| 1 		= 0
	if (data->config->reversed ^ (richtung == MOTOR_CW)) {
		gpio_clear(data->anschluss->direction_port, data->anschluss->direction);
	} else {
		gpio_set(data->anschluss->direction_port, data->anschluss->direction);
	}
}

void stepper_timer_init(motor_t *data) {

	gpio_mode_setup(data->anschluss->step_port, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, data->anschluss->step);
	gpio_set_af(data->anschluss->step_port, data->anschluss->af_number, data->anschluss->step);
	gpio_set_output_options(data->anschluss->step_port, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, data->anschluss->step);

	rcc_periph_reset_pulse(data->anschluss->timer_rcc);

	timer_set_mode(data->anschluss->timernumber, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_prescaler(data->anschluss->timernumber, 366); // 48000000 / (2 ^ (16-1))
	timer_continuous_mode(data->anschluss->timernumber);
	timer_set_period(data->anschluss->timernumber, 0xFFFF);

	timer_disable_oc_output(data->anschluss->timernumber, data->anschluss->timerchannel);

	timer_set_oc_mode(data->anschluss->timernumber, data->anschluss->timerchannel, TIM_OCM_TOGGLE);
	timer_disable_oc_clear(data->anschluss->timernumber, data->anschluss->timerchannel);
//	timer_enable_oc_preload(data->anschluss->timernumber, data->anschluss->timerchannel);
	timer_set_oc_value(data->anschluss->timernumber, data->anschluss->timerchannel, 0);
	timer_set_oc_polarity_high(data->anschluss->timernumber, data->anschluss->timerchannel);
	//timer_enable_oc_output(data->anschluss->timernumber, data->anschluss->timerchannel);

	//timer_enable_preload(data->anschluss->timernumber);
	timer_set_counter(data->anschluss->timernumber, 0);
	__asm__("DSB");
	nvic_set_priority(NVIC_TIM2_IRQ, 0);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	timer_enable_irq(data->anschluss->timernumber, TIM_DIER_CC3DE); // Update DMA request
	timer_set_dma_on_compare_event(data->anschluss->timernumber);
}

void stepper_dma_init(motor_t *data) {

	dma_disable_channel(data->anschluss->dmanumber, data->anschluss->dmachannel);

	nvic_set_priority(data->anschluss->dmainterrupt, 0);
	nvic_enable_irq(data->anschluss->dmainterrupt);

	// Reset DMA channels
	dma_channel_reset(data->anschluss->dmanumber, data->anschluss->dmachannel);

	// Set up tx dma
	dma_set_peripheral_address(data->anschluss->dmanumber, data->anschluss->dmachannel, (uint32_t) &TIM2_CCR3);
	dma_set_memory_address(data->anschluss->dmanumber, data->anschluss->dmachannel,
			(uint32_t) data->config->motor_tx_buf);
	dma_set_number_of_data(data->anschluss->dmanumber, data->anschluss->dmachannel, data->config->motor_tx_len);
	dma_set_read_from_memory(data->anschluss->dmanumber, data->anschluss->dmachannel);
	dma_enable_memory_increment_mode(data->anschluss->dmanumber, data->anschluss->dmachannel);
	dma_set_peripheral_size(data->anschluss->dmanumber, data->anschluss->dmachannel, DMA_CCR_PSIZE_32BIT);
	dma_set_memory_size(data->anschluss->dmanumber, data->anschluss->dmachannel,
	DMA_CCR_MSIZE_32BIT);
	dma_set_priority(data->anschluss->dmanumber, data->anschluss->dmachannel,
	DMA_CCR_PL_HIGH);

	dma_enable_circular_mode(data->anschluss->dmanumber, data->anschluss->dmachannel);
	dma_enable_transfer_complete_interrupt(data->anschluss->dmanumber, data->anschluss->dmachannel);
	dma_enable_half_transfer_interrupt(data->anschluss->dmanumber, data->anschluss->dmachannel);
	dma_enable_transfer_error_interrupt(data->anschluss->dmanumber, data->anschluss->dmachannel);
	dma_enable_channel(data->anschluss->dmanumber, data->anschluss->dmachannel);
}

static __inline__ float __attribute__((const)) reciprocal(float x) {
	union {
		float dbl;
		unsigned uint;
	} u;
	u.dbl = x;
	u.uint = (0xbe6eb3beU - u.uint) >> (unsigned char) 1;
	// pow( x, -0.5 )
	u.dbl *= u.dbl;                 // pow( pow(x,-0.5), 2 ) = pow( x, -1 ) = 1.0 / x
	return u.dbl;
}

void stepper_drive(motor_t *data, uint32_t *bufferstart) {
	/*
	 * Diese Funktion wird vom DMA Interrupt Handler aufgerufen. Abhängig davon, ob der Motor
	 * gerade läuft oder nicht werden hier PWM-Werte erzeugt und in den DMA-Puffer geschrieben.
	 *
	 * Diese Werte werden asynchron erzeugt und dann vom Timer synchron mit den Motorschritten abgearbeitet.
	 *
	 * Häufig benutze Variablen innerhalb der motor_t
	 * config->motor_tx_len 1, 0, 2
	 * config->motor_tx_buf 0, 0, 1
	 * data->speed_soll 2, 0, 1
	 * data->speed_actual 5, 0, 0
	 * data->config->accel 2, 4, 0
	 */
	float requiredSpeed = 0.0;
	uint32_t i = 0;
	int distanceToGo = 0;

	// Lösche halben Puffer.
	// Nicht notwendig, da extra Marken für Start / Stop vorhanden
	//memset(bufferstart, 0x0, (data->config->motor_tx_len >> 1) * sizeof(data->config->motor_tx_buf[0]));

	//NUR Wenn keine Fahrwege aktiv sind, da ansonsten die async des DMAs für noch nicht abgearbeitete Schritte die Drehrichtung ändert
	if (data->is_running == false) {
		stepper_set_direction(data, data->direction_actual);
	}

	distanceToGo = data->steps_soll - data->steps; //Anzahl an Schritten die für die aktuelle Bewegung noch übrig sind
	// targetPos != data->steps
	// Schleife durchläuft maximal eine viertel Pufferlänge, da für einen PWM-Zyklus zwei Werte
	// benötigt werden. (Timer im Toggle-Modus)

	for (; i < (data->config->motor_tx_len >> 2); i++) {

		if (distanceToGo == 0) {
			data->speed_actual = 0;
			data->is_running = false;
			//timer_disable_oc_output(data->anschluss->timernumber, data->anschluss->timerchannel); // Deaktivieren des Timers falls nicht gefahren werden soll
			break;// Verlasse Schleife und Funktion - Motor steht.

		} else if (distanceToGo > 0) { // CW -> positive Geschwindigkeit
			requiredSpeed = sqrtf((distanceToGo << 1) * data->config->accel);
			//timer_enable_oc_output(data->anschluss->timernumber, data->anschluss->timerchannel);
			data->is_running = true;
		} else { // CCW --> negative Geschwindigkeit
			requiredSpeed = -sqrtf(abs(distanceToGo << 1) * data->config->accel);
			//timer_enable_oc_output(data->anschluss->timernumber, data->anschluss->timerchannel);
			data->is_running = true;
		}

		if (requiredSpeed > data->speed_actual) { // Motor steht oder ist zu langsam

			// Need to accelerate in clockwise direction
			if (data->speed_actual == 0) {
				requiredSpeed = sqrtf(ldexpf(data->config->accel,1)); // anfahren
			} else {
				requiredSpeed = data->speed_actual + fabsf(data->config->accel * reciprocal(data->speed_actual)); // Beschleunigen
			}
			if (requiredSpeed > data->speed_soll) { // Maximalgeschwindigkeit erreicht
				requiredSpeed = data->speed_soll; // Limit
			}
		} else if (requiredSpeed < data->speed_actual) {

			// Need to accelerate in anticlockwise direction
			if (data->speed_actual == 0) {
				requiredSpeed = -sqrtf(ldexpf(data->config->accel,1));
			} else {
				requiredSpeed = data->speed_actual - fabsf(data->config->accel * reciprocal(data->speed_actual)); // bremsen
			}
			if (requiredSpeed < -data->speed_soll) {
				requiredSpeed = -data->speed_soll;
			}
		}

		data->speed_actual = requiredSpeed;
		distanceToGo--;
		//data->direction_actual == MOTOR_CW ? data->steps++ : data->steps--;

		// erzeuge und speichere Timerwerte:
		//uint16_t oneDivrequiredSpeed = (0xFFFF) * reciprocal(requiredSpeed);
		// Lösung 1
		float reci = reciprocal(requiredSpeed);
		uint16_t oneDivrequiredSpeed = ldexpf(reci,16) - reci;
		//Lösung 2
//		typedef union{
//			float f;
//			struct{
//				uint32_t man : 23;
//				uint8_t exp : 8;
//				uint8_t sign: 1;
//			} elem;
//		}float_t;
//		float reci = reciprocal(requiredSpeed);
//		float_t reci_mult = reci;
//		reci_mult.elem.exp += 16;
//		uint16_t oneDivrequiredSpeed = reci_mult.f;
		//#define MOTOR_C_DEBUG
#ifdef MOTOR_C_DEBUG
		usart_send_dec16(USART1, oneDivrequiredSpeed);
		usart_send_blocking(USART1, ',');
#endif
		size_t idx = i << 1;
		uint16_t tim1 = (uint16_t) (oneDivrequiredSpeed + data->last_timerval); // Wert kann maximal 0xFFFF annehmen
		bufferstart[idx] |= tim1;
		++idx;
		bufferstart[idx] |= (uint16_t) (tim1 + oneDivrequiredSpeed);
		data->last_timerval = (uint16_t) (tim1 + oneDivrequiredSpeed);
		if (distanceToGo == 0) {
			data->speed_actual = 0;
			//Dieses Paket stellt das Ende der Stepperdaten da
			fahrpfad_t dat;
			if (!motor_buffer_empty(data->fahrpfad_buf)) {
				// Eine weitere Fahrbewegung liegt vor
				motor_buffer_read(data->fahrpfad_buf, &dat);
				//Aktualisieren der aktuellen Steps
				data->steps_soll += dat.steps;
				//Aktualsieren der Geschwindigkeit
				data->speed_soll = dat.drive_speed;
				//Falls die Rotation sich ändert, neu setzen
				if (data->direction_actual != dat.rotat) {
					if (dat.rotat == MOTOR_CW) {
						bufferstart[(idx + 1) & (data->config->motor_tx_len - 1)] = STEPPER_CMD_ROT_CW;
					} else if (dat.rotat == MOTOR_CCW) {
						bufferstart[(idx + 1) & (data->config->motor_tx_len - 1)] = STEPPER_CMD_ROT_CCW;
					}
				}
				distanceToGo = data->steps_soll - data->steps + i +1;

			} else {
				//Falls der Buffer leer ist wird das STOP Bit gesetzt
				bufferstart[(idx + 1) & (data->config->motor_tx_len - 1)] = STEPPER_CMD_STOP;
			}
			timer_enable_irq(data->anschluss->timernumber, TIM_DIER_CC3IE);
		}

	}
	if (data->direction_actual == MOTOR_CW) {
		data->steps += i;
	} else {
		data->steps -= i;
	}
}

void stepper_new_cmd(motor_t* data, uint32_t* stepp_array, uint32_t offset) {
	//Timer cc3 irq deaktivieren
	fahrpfad_t dat;
	timer_disable_counter(data->anschluss->timernumber);
	timer_set_counter(data->anschluss->timernumber, 0);
	timer_disable_irq(data->anschluss->timernumber, TIM_DIER_CC3DE);
	TIM_SR(data->anschluss->timernumber) &= ~TIM_DIER_CC3DE; //Löschen von pending IRQ
	data->last_timerval = 0; // Reset, da unbekannte Zeit inzwischen vergangen
	if (!motor_buffer_empty(data->fahrpfad_buf)) {
		// Eine weitere Fahrbewegung liegt vor
		motor_buffer_read(data->fahrpfad_buf, &dat);
		//Aktualisieren der aktuellen Steps
		data->steps_soll += dat.steps;
		//Aktualsieren der Geschwindigkeit
		data->speed_soll = dat.drive_speed;
		data->direction_actual = dat.rotat;
	}
	stepper_drive(data, stepp_array);
	stepper_drive(data, &stepp_array[offset]);
	stepper_dma_init(data);
	//cc3 irq bit setzen (irq auslösen)
	timer_enable_irq(data->anschluss->timernumber, TIM_DIER_CC3DE);
	TIM_SR(data->anschluss->timernumber) |= TIM_DIER_CC3DE;
	timer_set_oc_value(data->anschluss->timernumber, data->anschluss->timerchannel, UINT16_MAX);
	//oc output aktivieren
	timer_enable_oc_output(data->anschluss->timernumber, data->anschluss->timerchannel);
	//timer starten (fester Versatz cc3 und timer start)
	timer_enable_counter(data->anschluss->timernumber);
}
