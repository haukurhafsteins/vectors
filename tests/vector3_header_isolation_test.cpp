#include "Vector3.hpp"

int main()
{
    const Vector3<int> vector(1, 2, 3);
    const auto values = vector.toArray();
    return values[0] == 1 && values[1] == 2 && values[2] == 3 ? 0 : 1;
}
