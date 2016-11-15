/*
 * data.h
 *
 *  Created on: 28 Sep 2016
 *      Author: mkadrummond
 */

#ifndef DATA_H_
#define DATA_H_

#include <string>
#include <vector>

namespace data {

	struct Length {
		int length;
		float pressure;

	};

	struct Measurement {

		int length;
		float pressure;
		int force;

	};

	class lengthFactory {
	public:
			lengthFactory();
			Length *createLength(int, float);
	};

	class measurementFactory {
	public:
			measurementFactory();
			Measurement *createMeasurement(int, float, int);
	};

	class programProfile {
	public:
		std::vector<Length*> vLengths;
		std::vector<Measurement*> vMeasurements;
	};


}



#endif /* DATA_H_ */
