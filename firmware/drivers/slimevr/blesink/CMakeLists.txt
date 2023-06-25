zephyr_library()

zephyr_library_sources_ifdef(CONFIG_BLESINK blesink.c)
