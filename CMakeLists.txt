cmake_minimum_required(VERSION 3.15)

project(stm32f401-led-strip-controller LANGUAGES C CXX ASM)

find_bsp(ID com.sysprogs.project_importers.stm32.cubemx SOURCE_PROJECT stm32f401-led-strip-controller.ioc)

add_bsp_based_executable(NAME stm32f401-led-strip-controller
	GENERATE_BIN
	GENERATE_MAP)

