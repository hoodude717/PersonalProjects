idf_component_register(SRCS cursor.c
                       INCLUDE_DIRS .
                       PRIV_REQUIRES joy pin
                       REQUIRES lcd)
# target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
