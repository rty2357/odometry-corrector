/*
 * odometry-corrector-conf.hpp
 *
 *  Created on: 2012/03/14
 *      Author: tyamada
 */

#ifndef ODOMETRY_CORRECTOR_CONF_HPP_
#define ODOMETRY_CORRECTOR_CONF_HPP_

#include <ssmtype/spur-odometry.h>
#include <ssmtype/pws-motor.h>
#include "ssm-laser.hpp"
#include "ssm-particles.hpp"


#include "gnd-observation-probability.hpp"
#include "gnd-configuration.hpp"
#include "gnd-lib-error.h"


/// structure declaration
// ---> namespace OdometryCorrector
namespace OdometryCorrector {
	/*!
	 * @brief process configuration
	 */
	struct proc_configuration;
} // <--- namespace OdometryCorrector



/// function declaration
// ---> namespace OdometryCorrector
namespace OdometryCorrector {

	int proc_conf_initialize(proc_configuration *c);
	int proc_conf_get(gnd::Conf::Configuration *src, proc_configuration* dest);
	int proc_conf_set(gnd::Conf::Configuration *dest, proc_configuration* src);
	int proc_conf_read(const char* f, proc_configuration* dest);
	int proc_conf_write(const char* f, proc_configuration* src);

} // <--- namespace OdometryCorrector


// constant definition
// ---> namespace OdometryCorrector
namespace OdometryCorrector {
#define SSM_CORRECTED_ODOMETRY_NAME "corrected-odometry"
	static const char proc_name[] = "odometry-corrector";
	static const char ssm_name[] = SSM_CORRECTED_ODOMETRY_NAME;

	// map file path
	static const gnd::Conf::parameter_array<char, 512> ConfIni_Map = {
			"map",
			"",		// file path
			"map file path"
	};


	// odometry kinematics parameter: right wheel radius
	static const gnd::Conf::parameter<double> ConfIni_WheelRadiusR = {
			"wheel-radius-r",
			0,		// right wheel radius
			"odometry kinematics parameter: right wheel radius"
	};

	// odometry kinematics parameter: left wheel radius
	static const gnd::Conf::parameter<double> ConfIni_WheelRadiusL = {
			"wheel-radius-l",
			0,		// left wheel radius
			"odometry kinematics parameter: left wheel radius"
	};

	// odometry kinematics parameter: tread
	static const gnd::Conf::parameter<double> ConfIni_Tread = {
			"tread",
			0,		// tread
			"odometry kinematics parameter: tread"
	};

	// odometry parameter: encoder resolution ( count per revolution )
	static const gnd::Conf::parameter<double> ConfIni_CountPerRevolution = {
			"count-per-rev",
			0,		// encoder resolution ( count per revolution )
			"odometry parameter: encoder resolution ( count per revolution )"
	};

	// odometry parameter: gear ratio
	static const gnd::Conf::parameter<double> ConfIni_Gear = {
			"gear-ratio",
			0,		// gear ratio
			"odometry parameter: gear ratio"
	};



	// name of wheel encoder shared data
	static const gnd::Conf::parameter_array<char, 32> ConfIni_WheelEncoderName = {
			"ssm-wheel-encoder-name",
			SNAME_PWS_MOTOR,		// shared data name
			"name of wheel encoder shared data"
	};

	// id of wheel encoder shared data
	static const gnd::Conf::parameter<int> ConfIni_WheelEncoderID = {
			"ssm-wheel-endoder-id",
			0,		// map file directory
			"id of wheel encoder shared data"
	};

	// id of proofed odometry shared data
	static const gnd::Conf::parameter_array<char, 32> ConfIni_OdometryErrorName = {
			"ssm-odometry-error-name",
			SNAME_ODOMETRY_ERR,		// shared data name
			"name of proofed odometry shared data"
	};

	// id of proofed odometry shared data
	static const gnd::Conf::parameter<int> ConfIni_OdometryErrorID = {
			"ssm-odometry-error-id",
			0,		// map file directory
			"id of proofed odometry shared data"
	};

	// name of position shared data
	static const gnd::Conf::parameter_array<char, 32> ConfIni_PositionName = {
			"ssm-position-name",
			SNAME_ODOMETRY,		// map file directory
			"name of odometry error estimation shared data"
	};

	// id of position shared data
	static const gnd::Conf::parameter<int> ConfIni_PositionID = {
			"ssm-position-id",
			0,		// map file directory
			"name of odometry error estimation shared data"
	};



} // <--- namespace OdometryCorrector


// type definition
// ---> namespace OdometryCorrector
namespace OdometryCorrector {

		/**
		 * @brief process configuration parameter
		 */
		struct proc_configuration {
			gnd::Conf::parameter_array<char, 512>	map;		///< @brief road surface map
			gnd::Conf::parameter<double>			radius_r;	///< @brief odometry kinematics parameter : right wheel radius
			gnd::Conf::parameter<double>			radius_l;	///< @brief odometry kinematics parameter : left wheel radius
			gnd::Conf::parameter<double>			tread;		///< @brief odometry kinematics parameter : tread
			gnd::Conf::parameter<double>			count_rev;	///< @brief odometry parameter : encoder resolution ( count per revolution )
			gnd::Conf::parameter<double>			gear;		///< @brief odometry parameter : gear ratio
			gnd::Conf::parameter_array<char, 32>	wenc_name;	///< @brief name of shared wheel encoder data
			gnd::Conf::parameter<int>				wenc_id;	///< @brief id of shared wheel encoder data
			gnd::Conf::parameter_array<char, 32>	odmerr_name;	///< @brief name of shared proofed odometry data
			gnd::Conf::parameter<int>				odmerr_id;	///< @brief id of shared proofed odometry data
			gnd::Conf::parameter_array<char, 32>	pos_name;	///< @brief name of shared position data
			gnd::Conf::parameter<int>				pos_id;		///< @brief id of shared position data

			proc_configuration();
		};

		/**
		 * @brief constructor
		 */
		inline
		proc_configuration::proc_configuration() {
			proc_conf_initialize(this);
		}
} // <--- namespace OdometryCorrector


// function definition
// ---> namespace OdometryCorrector
namespace OdometryCorrector {

		/**
		 * @brief initialize process configuration structure
		 * @param [in/out] conf : initialized
		 */
		inline
		int proc_conf_initialize(proc_configuration *conf) {
			gnd_assert(!conf, -1, "invalid null pointer");

			::memcpy(&conf->map,		&ConfIni_Map,					sizeof(ConfIni_Map));
			::memcpy(&conf->radius_r,	&ConfIni_WheelRadiusR,			sizeof(ConfIni_WheelRadiusR));
			::memcpy(&conf->radius_l,	&ConfIni_WheelRadiusL,			sizeof(ConfIni_WheelRadiusL));
			::memcpy(&conf->tread,		&ConfIni_Tread,					sizeof(ConfIni_Tread));
			::memcpy(&conf->count_rev,	&ConfIni_CountPerRevolution,	sizeof(ConfIni_CountPerRevolution));
			::memcpy(&conf->gear,		&ConfIni_Gear,					sizeof(ConfIni_Gear));
			::memcpy(&conf->wenc_id,	&ConfIni_WheelEncoderID,		sizeof(ConfIni_WheelEncoderID));
			::memcpy(&conf->wenc_name,	&ConfIni_WheelEncoderName,		sizeof(ConfIni_WheelEncoderName));
			::memcpy(&conf->wenc_id,	&ConfIni_WheelEncoderID,		sizeof(ConfIni_WheelEncoderID));
			::memcpy(&conf->odmerr_name,	&ConfIni_OdometryErrorName,	sizeof(ConfIni_OdometryErrorName));
			::memcpy(&conf->odmerr_id,	&ConfIni_OdometryErrorID,		sizeof(ConfIni_OdometryErrorID));
			::memcpy(&conf->pos_name,	&ConfIni_PositionName,			sizeof(ConfIni_PositionName));
			::memcpy(&conf->pos_id,		&ConfIni_PositionID,			sizeof(ConfIni_PositionID));
			return 0;
		}

		/**
		 * @brief get configuration parameter
		 * @param [in]  src  : configuration parameter declaration
		 * @param [out] dest : configuration parameter
		 */
		inline
		int proc_conf_get(gnd::Conf::Configuration *src, proc_configuration* dest) {
			gnd_assert(!src, -1, "invalid null pointer");
			gnd_assert(!dest, -1, "invalid null pointer");

			gnd::Conf::get_parameter(src, &dest->map);
			gnd::Conf::get_parameter(src, &dest->radius_r);
			gnd::Conf::get_parameter(src, &dest->radius_l);
			gnd::Conf::get_parameter(src, &dest->tread);
			gnd::Conf::get_parameter(src, &dest->count_rev);
			gnd::Conf::get_parameter(src, &dest->gear);
			gnd::Conf::get_parameter(src, &dest->wenc_name);
			gnd::Conf::get_parameter(src, &dest->wenc_id);
			gnd::Conf::get_parameter(src, &dest->odmerr_name);
			gnd::Conf::get_parameter(src, &dest->odmerr_id);
			gnd::Conf::get_parameter(src, &dest->pos_name);
			gnd::Conf::get_parameter(src, &dest->pos_id);
			return 0;
		}


		/**
		 * @brief set configuration parameter declaration
		 * @param [out] dest : configuration parameter declaration
		 * @param [in]  src  : configuration parameter
		 */
		inline
		int proc_conf_set(gnd::Conf::Configuration *dest, proc_configuration* src) {
			gnd_assert(!src, -1, "invalid null pointer");
			gnd_assert(!dest, -1, "invalid null pointer");

			gnd::Conf::set_parameter(dest, &src->map);
			gnd::Conf::set_parameter(dest, &src->radius_r);
			gnd::Conf::set_parameter(dest, &src->radius_l);
			gnd::Conf::set_parameter(dest, &src->tread);
			gnd::Conf::set_parameter(dest, &src->gear);
			gnd::Conf::set_parameter(dest, &src->count_rev);
			gnd::Conf::set_parameter(dest, &src->wenc_name);
			gnd::Conf::set_parameter(dest, &src->wenc_id);
			gnd::Conf::set_parameter(dest, &src->odmerr_name);
			gnd::Conf::set_parameter(dest, &src->odmerr_id);
			gnd::Conf::set_parameter(dest, &src->pos_name);
			gnd::Conf::set_parameter(dest, &src->pos_id);
			return 0;
		}

		/**
		 * @brief read configuration parameter file
		 * @param [in]  f    : configuration file name
		 * @param [out] dest : configuration parameter
		 */
		inline
		int proc_conf_read(const char* f, proc_configuration* dest) {
			gnd_assert(!f, -1, "invalid null pointer");
			gnd_assert(!dest, -1, "invalid null pointer");

			{ // ---> operation
				int ret;
				gnd::Conf::FileStream fs;
				// configuration file read
				if( (ret = fs.read(f)) < 0 )	return ret;

				return proc_conf_get(&fs, dest);
			} // <--- operation
		}

		/**
		 * @brief write configuration parameter file
		 * @param [in]  f  : configuration file name
		 * @param [in] src : configuration parameter
		 */
		inline
		int proc_conf_write(const char* f, proc_configuration* src){
			gnd_assert(!f, -1, "invalid null pointer");
			gnd_assert(!src, -1, "invalid null pointer");

			{ // ---> operation
				int ret;
				gnd::Conf::FileStream fs;
				// convert configuration declaration
				if( (ret = proc_conf_set(&fs, src)) < 0 ) return ret;

				return fs.write(f);
			} // <--- operation
		}

} // <--- OdometryCorrector


#endif /* OPSM_PARTICLE_EVALUATOR_CONF_HPP_ */
