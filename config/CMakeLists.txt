cmake_minimum_required(VERSION 3.10)

file(GLOB CONFIG_FILES *.yaml)
file(COPY ${CONFIG_FILES} DESTINATION .)

configure_file(default.yaml default.yaml COPYONLY)