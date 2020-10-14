//----------------------------------------------------------------------------------------------------------------------
// GRVC UAL
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2016 GRVC University of Seville
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------
#include <fixed_wing_lib/fixed_wing.h>

class InheritanceTestClass : public grvc::fw_ns::FixedWing {
  public:
   InheritanceTestClass() {
       std::cout << " InheritanceTestClass running " << std::endl;
   }
   void printTest() {
       std::cout << test_ << std::endl;
   }

  protected:
   std::string test_ = "test";
};

int main(int _argc, char** _argv) {

    ros::init(_argc, _argv, "fixed_wing_node");

    // InheritanceTestClass inheritance_test_class;    // Working just the same.
    // inheritance_test_class.printMission();
    // inheritance_test_class.printTest();

    grvc::fw_ns::FixedWing fw;
    fw.printMission();

    while (ros::ok()) { sleep(1); }

    return 0;
}