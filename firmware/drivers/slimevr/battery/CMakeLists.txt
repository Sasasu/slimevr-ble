zephyr_library()

zephyr_library_sources_ifdef(CONFIG_ADC_FUEL_GAUGE adc_fuel_gauge.c)
