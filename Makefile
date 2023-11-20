# Project Name
TARGET = semester_project

# Sources
CPP_SOURCES = semester_project.cpp

# Library Locations
LIBDAISY_DIR = ../../libdaisy/
DAISYSP_DIR = ../../daisysp/

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
