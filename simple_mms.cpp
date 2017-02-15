#include <mms/map.h>
#include <mms/vector.h>
#include <mms/writer.h>

#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/types.h>

template<class P>
struct NodeT {
  uint64_t data_;

  // Expose struct's fields to mms
  template<class A> void traverseFields(A a) const { a(data_); }
};

using Node = NodeT<mms::Mmapped>;

template<class T>
using Vector = mms::vector<mms::Mmapped, T>;

int main() {


    mms::vector<mms::Standalone, NodeT<mms::Standalone>> test;
    test.push_back({55});
    test.push_back({15});

    // Serialize
    std::ofstream out("test_cache");
    size_t pos = mms::write(out, test);
    out.close();

    // mmap data
    int fd = ::open("test_cache", O_RDONLY);
    struct stat st;
    fstat(fd, &st);
    char* data = (char*) mmap(0, st.st_size, PROT_READ, MAP_SHARED, fd, 0);
    const Vector<Node>* m_test =
        reinterpret_cast<const Vector<Node>*>(data + pos);

    std::cout << "hi " << m_test->at(0).data_;
    return 0;
}
