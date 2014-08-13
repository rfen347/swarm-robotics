#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
using namespace std;

//Time variables for alpha
class timer {
	private:
		unsigned long begTime;
	public:
		void start() {
			begTime = clock();
		}

		unsigned long elapsedTime() {
			return ((unsigned long) clock() - begTime) / CLOCKS_PER_SEC;
		}

		bool isTimeout(unsigned long seconds) {
			return seconds >= elapsedTime();
		}

		int wakeTime=2;
		int eatingTime1 = 8;
		int eatingTime2 = 14;
};

//Functions
//wakeUp(Resident: resident):void
// Need to call- wakeUp()

//eatingTime(Resident: resident):void
// Need to call -getReadyToEat()->cooking() (robot will spin)->eat()

int main() {
	//create timer, start timer
	timer t;
	t.start();
	cout << "timer started . . ." << endl;
	while(true) {
		if(t.elapsedTime() ==t.wakeTime) {
			cout<<"Resident wakes up"<<endl;
			break;
		}
	}
	while(true) {
		if(t.elapsedTime() == t.eatingTime1) {
			cout<<"Resident eats"<<endl;
			break;
				}
	}
	while(true) {
			if(t.elapsedTime() == t.eatingTime2) {
				cout<<"Resident eats"<<endl;
				break;
					}
		}
	cout << t.elapsedTime() <<  " seconds elapsed" << endl;
}





