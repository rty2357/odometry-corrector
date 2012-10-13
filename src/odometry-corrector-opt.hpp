/*
 * odometry-corrector-opt.hpp
 *
 *  Created on: 2012/03/14
 *      Author: tyamada
 */

#ifndef ODOMETRY_CORRECTOR_OPT_HPP_
#define ODOMETRY_CORRECTOR_OPT_HPP_

#include <string.h>
#include <unistd.h>
#include <getopt.h>

#include "odometry-corrector-conf.hpp"

// type declaration
// ---> namespace OdometryCorrector
namespace OdometryCorrector {
	class proc_option_reader;
}// <--- namespace OdometryCorrector


// constant declaration
// ---> namespace OdometryCorrector
namespace OdometryCorrector {
	const char ConfFile[] = "odometry-corrector.conf";
	const char ShortOpt[] = "hg:G::m:e:";
	const struct option LongOpt[] = {
			{"help", 							no_argument,		0,	'h'},
			{"config",							required_argument,	0,	'g'},
			{"write-config",					optional_argument,	0,	'G'},
			{ConfIni_Map.token,					required_argument,	0,	'm'},
			{ConfIni_WheelEncoderID.token,		required_argument,	0,	'e'},
			{ConfIni_OdometryErrorID.token,		required_argument,	0,	'o'},
			{ConfIni_PositionID.token,			required_argument,	0,	'p'},
			{0, 0, 0, 0}	// end of array
	};
}// <--- namespace OdometryCorrector



// type definition
// ---> namespace OdometryCorrector
namespace OdometryCorrector {
	class proc_option_reader {
		// ---> return value definition
	public:
		static const int RetFail = -1;
		static const int RetHelp = 1;
		static const int RetWriteConf = 2;

	public:

		// ---> constructor
	public:
		proc_option_reader();
		proc_option_reader(proc_configuration *c);
		~proc_option_reader();
	private:
		proc_configuration *conf;

		// set storage
	public:
		int set(proc_configuration *c);

		// read
	public:
		bool read(int argc, char **argv);

	};

	/**
	 * @brief constructor
	 */
	inline
	proc_option_reader::proc_option_reader(): conf(0) {
	}

	/**
	 * @brief constructor
	 */
	inline
	proc_option_reader::proc_option_reader(proc_configuration *c): conf(c) {
	}

	/**
	 * @brief destructor
	 */
	inline
	proc_option_reader::~proc_option_reader() {
	}

	/**
	 * @brief set configuration parameter storage
	 */
	inline
	int proc_option_reader::set(proc_configuration *c) {
		conf = c;
		return 0;
	}


	/**
	 * @brief read option
	 */
	inline
	bool proc_option_reader::read(int argc, char **argv)
	{
		gnd_error(!conf, RetFail, "Invalid Member");

		while(1){
			int opt;
			optarg = 0;
			opt = ::getopt_long(argc, argv, ShortOpt, LongOpt, 0);
			if(opt < 0)	break;

			switch(opt){

			// read configure
			case 'g':
			{
				if( proc_conf_read(optarg, conf) < 0){
					::fprintf(stderr, " ... [\x1b[1m\x1b[31mERROR\x1b[30m\x1b[0m]: -g option, configure file syntax error\n");
					return RetFail;
				}
			} break;

			// write configure
			case 'G': {
				proc_conf_write( optarg ? optarg : ConfFile, conf);
				::fprintf(stderr, " ... output configuration file \"\x1b[4m%s\x1b[0m\"\n", optarg ? optarg : ConfFile);
			} return RetWriteConf;

			// entry map file
			case 'm': ::strcpy(conf->map.value, optarg);		break;
			// entry id of shared encoder data
			case 'e': conf->wenc_id.value = ::atoi(optarg);		break;
			// entry id of shared position data
			case 'p': conf->odmerr_id.value = ::atoi(optarg);		break;
			// entry id of shared proofed odometry data
			case 'o': conf->pos_id.value = ::atoi(optarg);		break;

			// show help
			case 'h':
			{
				int i = 0;
				fprintf(stderr, "\t\x1b[1mNAME\x1b[0m\n");
				fprintf(stderr, "\t\t\x1b[1m%s\x1b[0m - observation probability scan matching optimizer\n", proc_name);
				fprintf(stderr, "\n");

				fprintf(stderr, "\t\x1b[1mSYNAPSIS\x1b[0m\n");
				fprintf(stderr, "\t\t\x1b[1m%s\x1b[0m [\x1b[4mOPTIONS\x1b[0m]\n", proc_name);
				fprintf(stderr, "\n");

				fprintf(stderr, "\t\x1b[1mDISCRIPTION\x1b[0m\n");
				fprintf(stderr, "\t\t\x1b[1m%s\x1b[0m is estimate the robot position and optimize it with newton method.\n", proc_name);

				fprintf(stderr, "\n");
				fprintf(stderr, "\t\x1b[1mOPTIONS\x1b[0m\n");
				fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m, \x1b[1m--%s\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
				fprintf(stderr, "\t\t\tprint help\n");
				fprintf(stderr, "\n");
				i++;

				fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m, \x1b[1m--%s\x1b[0m\n", LongOpt[i].val,  LongOpt[i].name);
				fprintf(stderr, "\t\t\tread configure file\n");
				fprintf(stderr, "\n");
				i++;

				fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m, \x1b[1m--%s\x1b[0m\n", LongOpt[i].val,  LongOpt[i].name);
				fprintf(stderr, "\t\t\twirte configure file\n");
				fprintf(stderr, "\n");
				i++;

				fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m, \x1b[1m--%s\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
				fprintf(stderr, "\t\t\tinput map files.\n");
				fprintf(stderr, "\n");
				i++;

				fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m, \x1b[1m--%s\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
				fprintf(stderr, "\t\t\tentry ssm-data wheel encoder id\n");
				fprintf(stderr, "\n");
				i++;

				fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m, \x1b[1m--%s\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
				fprintf(stderr, "\t\t\tentry ssm-data proofed odometry id\n");
				fprintf(stderr, "\n");
				i++;

				fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m, \x1b[1m--%s\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
				fprintf(stderr, "\t\t\tentry ssm-data wheel encoder id\n");
				fprintf(stderr, "\n");
				i++;

				return RetHelp;
			}break;
			}
		}
		return 0;
	}

}// <--- namespace OdometryCorrector



#endif /* SERP_ODOMETRY_OPT_HPP_ */
