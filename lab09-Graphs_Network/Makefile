# Configure compiler settings
CXX=clang++
CXXFLAGS=-g -std=c++11 -Werror -D_GLIBCXX_DEBUG
# The object files for the program.
OFILES = stringUtils.o
# The header files for the program
HFILES = $(shell ls -1 *.h)

# unittest++ keeps its object files in this library.
UNITTEST_LIB = -lUnitTest++

all: oinkedIn bacon instaham tests manualTests WrittenLab.pdf

# This target builds oinkedIn.
oinkedIn: $(HFILES) $(OFILES) oinkedIn.o
	$(CXX) $(CXXFLAGS) -o $@ oinkedIn.o $(OFILES)

# This target builds bacon.
bacon: $(HFILES) $(OFILES) bacon.o
	$(CXX) $(CXXFLAGS) -o $@ bacon.o $(OFILES)

# This target builds instaham.
instaham: $(HFILES) $(OFILES) instaham.o
	$(CXX) $(CXXFLAGS) -o $@ instaham.o $(OFILES)

# This target builds your tests.
tests: $(HFILES) $(OFILES) tests.o
	$(CXX) $(CXXFLAGS) -o $@ tests.o $(OFILES) $(UNITTEST_LIB)

# This target builds your manual testing program.
manualTests: $(HFILES) $(OFILES) manualTests.o
	$(CXX) $(CXXFLAGS) -o $@ manualTests.o $(OFILES)

# This target describes how to compile a .o file from a .cpp file.
%.o: %.cpp $(HFILES)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# This target compiles your LaTeX file.
WrittenLab.pdf: WrittenLab.tex
	pdflatex WrittenLab.tex

# This target deletes the temporary files we have built.
.PHONY: clean
clean:
	rm -f *.o
	rm -f gui/*.o
	rm -f inroads
	rm -f tests
	rm -f manualTests
