#include "Exception.hpp"

using namespace ds;

ostream &operator<<(ostream &os, ds::Exception &e) {
    os << "Exception caught in file \"" << e.file << "\", on line " << e.line << ", with error " << e.err << ": " << e.msg;
    return os;
}

