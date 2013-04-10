.SUFFIXES: .cpp .hpp

# Programs
SHELL 	= bash
CC     	= g++
LD	= ld
RM 	= rm
ECHO	= /bin/echo
CAT	= cat
PRINTF	= printf
SED	= sed
DOXYGEN = doxygen
######################################
# Project Name (generate executable with this name)
TARGET = cs296_base

# Project Paths
INSTALL_PATH = ~/
PROJECT_ROOT=$(CURDIR)
EXTERNAL_ROOT=$(PROJECT_ROOT)/external
SRCDIR = $(PROJECT_ROOT)/src
OBJDIR = $(PROJECT_ROOT)/obj
BINDIR = $(PROJECT_ROOT)/bin
DOCDIR = $(PROJECT_ROOT)/doc
SCRIPTSDIR = $(PROJECT_ROOT)/scripts
NON_SRCDIR = $(PROJECT_ROOT)/src_non_gui

# Library Paths
BOX2D_ROOT=$(EXTERNAL_ROOT)
GLUI_ROOT=/usr
GL_ROOT=/usr

#Libraries
LIBS = -lBox2D -lglui -lglut -lGLU -lGL

# Compiler and Linker flags
CPPFLAGS =-g -O3 -Wall 
CPPFLAGS+=-I $(BOX2D_ROOT)/include -I $(GLUI_ROOT)/include
LDFLAGS= -pg -L $(BOX2D_ROOT)/lib -L $(GLUI_ROOT)/lib

######################################

NO_COLOR=\e[0m
OK_COLOR=\e[1;32m
ERR_COLOR=\e[1;31m
WARN_COLOR=\e[1;33m
MESG_COLOR=\e[1;34m
FILE_COLOR=\e[1;37m

OK_STRING="[OK]"
ERR_STRING="[ERRORS]"
WARN_STRING="[WARNINGS]"
OK_FMT="${OK_COLOR}%30s\n${NO_COLOR}"
ERR_FMT="${ERR_COLOR}%30s\n${NO_COLOR}"
WARN_FMT="${WARN_COLOR}%30s\n${NO_COLOR}"
######################################

SRCS := $(wildcard $(SRCDIR)/*.cpp)
INCS := $(wildcard $(SRCDIR)/*.hpp)
OBJS := $(SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)


.PHONY: all setup doc clean distclean

all: setup $(BINDIR)/$(TARGET)

setup:
	@$(ECHO) "Setting up compilation..."
	@mkdir -p obj
	@mkdir -p bin
	@mkdir -p plots
	@mkdir -p data

	

install_gui: setup $(BINDIR)/$(TARGET)

install_helper:
	make release_prof
	make debug_prof


install: setup install_helper install_non_gui report 
	make install_gui
	@cd ..; cp -r cs296_base_code $(INSTALL_PATH)/
	make clean
	


non_gui_helper:
	@mkdir temp
	@mv $(SRCDIR)/cs296_base.cpp temp/
	@mv $(SRCDIR)/cs296_base.hpp temp/
	@mv $(SRCDIR)/main.cpp temp/
	@cp $(NON_SRCDIR)/* $(SRCDIR)/

revert_back:
	@mv $(BINDIR)/cs296_base $(BINDIR)/cs296_base_no_gui
	@cp temp/* $(SRCDIR)/
	@rm -rf temp

install_non_gui: setup non_gui_helper $(BINDIR)/$(TARGET) revert_back

	  

$(BINDIR)/$(TARGET): $(OBJS)
	@$(PRINTF) "$(MESG_COLOR)Building executable:$(NO_COLOR) $(FILE_COLOR) %16s$(NO_COLOR)" "$(notdir $@)"
	@$(CC) -o $@ $(LDFLAGS) $(OBJS) $(LIBS) 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else $(PRINTF) $(OK_FMT) $(OK_STRING); \
	fi;
	@$(RM) -f temp.log temp.err

-include $(OBJS:.o=.d)

$(OBJS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	@$(PRINTF) "$(MESG_COLOR)Compiling: $(NO_COLOR) $(FILE_COLOR) %25s$(NO_COLOR)" "$(notdir $<)"
	@$(CC) $(CPPFLAGS) -c $< -o $@ -MD 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else printf "${OK_COLOR}%30s\n${NO_COLOR}" "[OK]"; \
	fi;
	@$(RM) -f temp.log temp.err
	
	
$(EXTERNAL_ROOT)/lib/libBox2D.a:
	@ rm -rf $(EXTERNAL_ROOT)/src/Box2D
	@ rm -rf $(EXTERNAL_ROOT)/include/*
	@ rm -rf $(EXTERNAL_ROOT)/lib/*
	@ tar zxf $(EXTERNAL_ROOT)/src/Box2D.tgz
	@ mv Box2D $(EXTERNAL_ROOT)/src
	@ mkdir $(EXTERNAL_ROOT)/src/Box2D/build296
	@ cd $(EXTERNAL_ROOT)/src/Box2D/build296; cmake ../.
	@ cd $(EXTERNAL_ROOT)/src/Box2D/build296; make; make install


	
report:$(EXTERNAL_ROOT)/lib/libBox2D.a install_non_gui
	@$(ECHO) -e "Generating CSV..."
	@python3 $(SCRIPTSDIR)/g18_gen_csv.py
	@$(ECHO) -e "CSV Generated"
	@python3 $(SCRIPTSDIR)/g18_gen_html.py
	@$(ECHO) -e "HTML Report Generated"
	@python3 $(SCRIPTSDIR)/g18_gen_plots.py
	@$(ECHO) -e "Plots Generated"
	cd doc/ ; pdflatex g18_prof_report.tex ; bibtex g18_prof_report ; pdflatex g18_prof_report.tex ; pdflatex g18_prof_report.tex
	@ rm -f doc/g18_prof_report.aux
	@ rm -f doc/g18_prof_report.bbl
	@ rm -f doc/g18_prof_report.blg
	@ rm -f doc/g18_prof_report.dvi
	@ rm -f doc/g18_prof_report.log
	@ mv doc/g18_prof_report.pdf data/
	@$(ECHO) -e "Done"
	
	
debug_helper:
	@$(RM) -rf $(EXTERNAL_ROOT)/include/*
	@$(RM) -rf $(EXTERNAL_ROOT)/lib/*
	@$(RM) -rf $(EXTERNAL_ROOT)/src/Box2D
	@$(RM) -rf $(BINDIR)/*
	@ tar zxf $(EXTERNAL_ROOT)/src/Box2D.tgz
	@ mv Box2D $(EXTERNAL_ROOT)/src
	@ mkdir $(EXTERNAL_ROOT)/src/Box2D/build296
	@ cd $(EXTERNAL_ROOT)/src/Box2D/build296; cmake -DCMAKE_BUILD_TYPE=Debug ../.
	@ cd $(EXTERNAL_ROOT)/src/Box2D/build296; make; make install



debug_prof:CPPFLAGS= -pg -Wall -I $(BOX2D_ROOT)/include -I $(GLUI_ROOT)/include
debug_prof: debug_helper setup install_non_gui
	@ $(BINDIR)/cs296_base_no_gui 10000
	@gprof $(BINDIR)/cs296_base_no_gui gmon.out > $(DOCDIR)/g18_debug_prof.dat
	@ cd doc; chmod +x gprof2dot.py
	@ cd $(PROJECT_ROOT); gprof $(BINDIR)/cs296_base_no_gui | ./doc/gprof2dot.py | dot -Tpng -o doc/debug.png
	@ mv doc/g18_debug_prof.dat data/
	@ mv doc/debug.png data/
	

release_helper:
	@$(RM) -rf $(EXTERNAL_ROOT)/include/*
	@$(RM) -rf $(EXTERNAL_ROOT)/lib/*
	@$(RM) -rf $(EXTERNAL_ROOT)/src/Box2D
	@$(RM) -rf $(BINDIR)/*
	@ tar zxf $(EXTERNAL_ROOT)/src/Box2D.tgz
	@ mv Box2D $(EXTERNAL_ROOT)/src
	@ mkdir $(EXTERNAL_ROOT)/src/Box2D/build296
	@ cd $(EXTERNAL_ROOT)/src/Box2D/build296; cmake -DCMAKE_BUILD_TYPE=Release ../.
	@ cd $(EXTERNAL_ROOT)/src/Box2D/build296; make;  make install

release_prof: CPPFLAGS = -O3 -pg -Wall -I $(BOX2D_ROOT)/include -I $(GLUI_ROOT)/include
release_prof: release_helper setup install_non_gui
	@ $(BINDIR)/cs296_base_no_gui 10000
	@gprof $(BINDIR)/cs296_base_no_gui gmon.out > $(DOCDIR)/g18_release_prof.dat
	@ cd doc; chmod +x gprof2dot.py
	@ cd $(PROJECT_ROOT); gprof $(BINDIR)/cs296_base_no_gui | ./doc/gprof2dot.py | dot -Tpng -o doc/release.png
	@ mkdir -p data
	@ mv doc/g18_release_prof.dat data/
	@ mv doc/release.png data/



doc:
	@$(ECHO) -n "Generating Doxygen Documentation ...  "
	@$(RM) -rf doc/html
	@$(DOXYGEN) $(DOCDIR)/Doxyfile 2 > /dev/null
	@$(ECHO) "Done"

prof_clean:
	@rm -rf bin
	@rm -rf data
	@rm -rf obj
	@rm -rf plots
	@rm -f gmon.out
	@rm -f cs296_g18_project.tar.gz

clean:prof_clean
	@rm -rf $(DOCDIR)/html
	@rm -f $(DOCDIR)/g18_report.html
	
dist_clean:clean
	@rm -rf $(EXTERNAL_ROOT)/include/*
	@rm -rf $(EXTERNAL_ROOT)/lib/*
	@rm -rf $(EXTERNAL_ROOT)/src/Box2D


dist	: dist_clean
	@cd ..; tar -zcf cs296_g18_project.tar.gz cs296_base_code
