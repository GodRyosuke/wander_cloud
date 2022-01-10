#include "testInterface.h"

testClass::testClass()
    :result(0)
{}

void testClass::adder(int a, int b)
{
    this->result = a + b;
}


void testClass::mul(int a, int b)
{
    this->result = a * b;
}