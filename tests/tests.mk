TESTS_CXXFLAGS = $(CXXFLAGS) -Ilib/catch/include

genfiles:
	mkdir genfiles

genfiles/main_test.o: tests/main.cpp
	$(CXX) $(TESTS_CXXFLAGS) -c $< -o $@

define GENERIC_TEST

genfiles/$(1).o: tests/$(1).cpp genfiles
	$(CXX) $(TESTS_CXXFLAGS) -c $$< -o $$@

genfiles/$(1): genfiles/$(1).o genfiles/main_test.o
	$(CXX) $(LDFLAGS) $$^ -o $$@

test:: genfiles/$(1)
	$$<

endef

$(foreach test,$(TESTS),$(eval $(call GENERIC_TEST,$(test))))
