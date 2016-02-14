TESTS_CXXFLAGS = $(CXXFLAGS) -Ivendor/catch/include

genfiles:
	mkdir genfiles

genfiles/main_test.o: tests/main.cpp
	$(CXX) $(TESTS_CXXFLAGS) -c $< -o $@

test:: genfiles

define GENERIC_TEST

genfiles/$(1).o: tests/$(1).cpp lib/*.h
	$(CXX) $(TESTS_CXXFLAGS) -c $$< -o $$@

genfiles/$(1): genfiles/$(1).o genfiles/main_test.o lib/types.o
	$(CXX) $(LDFLAGS) $$^ -o $$@

.PHONY: test
test:: genfiles/$(1)
	$$< ||:


endef

$(foreach test,$(TESTS),$(eval $(call GENERIC_TEST,$(test))))
