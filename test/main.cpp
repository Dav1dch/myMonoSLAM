#include <iostream>
#include <set>
using namespace std;

#include <iostream>

using namespace std;

int main(int argc, char **argv) {

    std::set<int> s1;
    std::cout << (s1.find(2) != s1.end()) << std::endl;

    return 0;
}
