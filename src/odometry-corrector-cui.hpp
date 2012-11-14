/*
 * odometry-coorector-cui.hpp
 *
 *  Created on: 2012/03/21
 *      Author: tyamada
 */

#ifndef ODOMETRY_CORRECTOR_CUI_HPP_
#define ODOMETRY_CORRECTOR_CUI_HPP_

#include "gnd-cui.hpp"
#include "gnd-util.h"

namespace OdometryCorrector {

	static const gnd::cui_command cui_cmd[] = {
			{"Quit",					'Q',	"shut off process"},
			{"help",					'h',	"show help"},
			{"show",					's',	"state show mode"},
			{"", '\0'}
	};

	static const double CuiShowCycle = gnd_sec2time(1.0);
}
#endif /*  ODOMETRY_CORRECTOR_CUI_HPP_ */
