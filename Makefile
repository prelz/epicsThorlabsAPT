#Makefile at top of application tree
# "#!" marks lines that can be uncommented.

TOP = .
include $(TOP)/configure/CONFIG

DIRS += configure
ThorlabsAPTApp_DEPEND_DIRS   = configure

DIRS += ThorlabsAPTApp

include $(TOP)/configure/RULES_TOP
