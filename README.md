# EE186 Lab 4

---

## Part 2

1.  
For the STM32 ADC, the fast-channel throughput depends on the chosen resolution. Approximate maximum rates are:

- 12-bit: ~5.33 MSPS  
- 10-bit: ~6.15 MSPS  
- 8-bit:  ~7.27 MSPS  
- 6-bit:  ~8.88 MSPS

2.  
Since the STM32 ADC senses voltage rather than resistance, a voltage divider is used with the light-sensitive element. Changes in its resistance are translated into a proportional voltage that stays within the converter’s input range. Without the divider, the ADC input could float and yield erratic, noisy readings.

---

## Part 3

3.  
A higher DAC sampling rate places more points on each cycle of the waveform, which makes the analog output smoother and closer to the intended shape. If the rate is too low, the output appears stepped, injecting extra high-frequency components and reducing fidelity.

On how they sound:
- Sine wave: a clean, pure tone with only the fundamental present (tuning-fork like).  
- Square wave: a buzzy, harsh timbre because it contains a series of odd harmonics that fall off with frequency.  
- Sawtooth wave: a bright, rich tone since both odd and even harmonics are present.

A series capacitor is included in the audio path to block the DAC’s DC offset (typically around Vref/2) while allowing the AC audio signal to pass. Together with the load, this forms a high-pass network that recenters the signal around 0 V and keeps DC away from the speaker.
