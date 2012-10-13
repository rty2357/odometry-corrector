//============================================================================
// Name        : odometry-corrector.cpp
// Author      : tyamada
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <stdio.h>
#include <time.h>

#include <ypspur.h>
#include <ssmtype/spur-odometry.h>
#include <ssmtype/pws-motor.h>

#include "ssm-odometry-err.hpp"

#include "odometry-corrector-opt.hpp"
#include "odometry-corrector-conf.hpp"
#include "odometry-corrector-cui.hpp"

#include "gnd-time.hpp"
#include "gnd-odometry-correction.hpp"
#include "gnd-shutoff.hpp"

int main(int argc, char* argv[], char *env[]) {
	SSMApi<PWSMotor>					ssm_wenc;		// shared wheel encoder data (read)
	SSMApi<Spur_Odometry>				ssm_pos;		// shared position data (read)
	SSMApi_OdometryErr					ssm_odmerr;		// shared odometry error data (write)

	gnd::odometry::cmap					map;			// road surface environment map

	struct {
		double radius_r;
		double radius_l;
		double tread;
		double count_rev;
		double gear;
	} odm_knm;											// odometry kinematics parameter

	OdometryCorrector::proc_configuration	pconf;			// process configuration
	OdometryCorrector::proc_option_reader	popt(&pconf);	// process option reader
	gnd::cui								pcui;			// cui



	{ // ---> initialize
		int ret;
		size_t phase = 1;


		// ---> read process options
		if( (ret = popt.read(argc, argv)) != 0 ){
			return ret;
		} // <--- read process options



		{ // ---> allocate SIGINT to shut-off
			::proc_shutoff_clear();
			::proc_shutoff_alloc_signal(SIGINT);
		} // <--- allocate SIGINT to shut-off



		{ // ---> show initialize task
			::fprintf(stderr, "==========Initialize==========\n");
			::fprintf(stderr, " %d. read road surface environment map \"%s\"\n", phase++, pconf.map.value);
			::fprintf(stderr, " %d. set odometry kinematics parameter\n", phase++ );
			::fprintf(stderr, " %d. initialize ssm\n", phase++ );
			::fprintf(stderr, " %d. open shared wheel encoder data named \"\x1b[4m%s\x1b[0m\"\n", phase++, pconf.pos_name.value );
			::fprintf(stderr, " %d. open shared position data named \"\x1b[4m%s\x1b[0m\"\n", phase++, pconf.pos_name.value );
			::fprintf(stderr, " %d. create shared proofed odometry data with road surface environmental map named \"\x1b[4m%s\x1b[0m\"\n", phase++, pconf.odmerr_name.value );
			::fprintf(stderr, "\n\n");
		} // <--- show initialize task





		// ---> read road surface environmental map
		if( !::is_proc_shutoff() ) {
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => read road surface environment map \"%s\"", pconf.map.value);

			if( *pconf.map.value == '\0' ) {
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: missing map path\n");
				::proc_shutoff();
			}
			else if( map.fread(pconf.map.value) < 0) {
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to read map\n");
				::proc_shutoff();
			}
			else {
				::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m\n");
			}
		} // <--- read road surface environmental map


		// ---> set odometry kinematics parameter
		if( !::is_proc_shutoff() ) {
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => set odometry kinematics parameter\n");

			{ // ---> entry from configuration parameter
				odm_knm.radius_r = pconf.radius_r.value;
				odm_knm.radius_l = pconf.radius_l.value;
				odm_knm.tread = pconf.tread.value;
				odm_knm.count_rev = pconf.count_rev.value;
				odm_knm.gear = pconf.gear.value;
			} // <--- entry from configuration parameter

			// ---> get unsetted parameter from ypspur-coordinator
			if( odm_knm.radius_r <= 0 || odm_knm.radius_l <= 0 || odm_knm.tread <= 0 ||
				odm_knm.count_rev <= 0 || odm_knm.gear <=0 ) {
				::fprintf(stderr, " ... some paremter is unsetted\n");
				::fprintf(stderr, " ... get from ypspur-coordinator\n");

				// ---> initialize ypspur
				if( Spur_init() < 0 ) {
					::fprintf(stderr, "  ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to connect ypspur-coordinator.\n");
					::proc_shutoff();
				} // <--- initialize ypspur
				else { // ---> get odometry kinematics parameter
					// get kinmeatics from ypspur-coordinator
					if( !::is_proc_shutoff() && odm_knm.radius_r <= 0 && YP_get_parameter( YP_PARAM_RADIUS_R, &odm_knm.radius_r ) < 0 ){
						::proc_shutoff();
					}
					if( !::is_proc_shutoff() && odm_knm.radius_l <= 0 && YP_get_parameter( YP_PARAM_RADIUS_L, &odm_knm.radius_l ) < 0 ){
						::proc_shutoff();
					}
					if( !::is_proc_shutoff() && odm_knm.tread <= 0 && YP_get_parameter( YP_PARAM_TREAD, &odm_knm.tread ) < 0 ){
						::proc_shutoff();
					}
					if( !::is_proc_shutoff() && odm_knm.gear <= 0 && YP_get_parameter(YP_PARAM_COUNT_REV, &odm_knm.count_rev) < 0 ){
						::proc_shutoff();
					}
					if( !::is_proc_shutoff() && odm_knm.count_rev <= 0 && YP_get_parameter(YP_PARAM_GEAR, &odm_knm.gear) < 0 ){
						::proc_shutoff();
					}
				} // <--- get odometry kinematics parameter


			} // <--- get unsetted parameter from ypspur-coordinator

			::fprintf(stderr, " ... right wheel radius: %lf\n", odm_knm.radius_r);
			::fprintf(stderr, " ...  left wheel radius: %lf\n", odm_knm.radius_l);
			::fprintf(stderr, " ...              tread: %lf\n", odm_knm.tread);
			::fprintf(stderr, " ... encoder resolution: %lf\n", odm_knm.count_rev);
			::fprintf(stderr, " ...         gear ratio: %lf\n", odm_knm.gear);


			if( odm_knm.radius_r > 0 && odm_knm.radius_l > 0 && odm_knm.tread > 0 &&
				odm_knm.count_rev > 0 && odm_knm.gear > 0 ) {
				::fprintf(stderr, " ... \x1b[1mOK\x1b[0m\n");
			}
			else {
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to get odometry kinematics parameter\n");
				::proc_shutoff();
			}

		} // <--- set odometry kinematics parameter


		// ---> initialize ssm
		if( !::is_proc_shutoff() ){
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => initialize ssm\n");

			if( !initSSM() ){
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to initialize ssm.\n");
				::proc_shutoff();
			}
			else {
				::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m\n");
			}
		} // <--- initialize ssm

		// ---> create proofed odometry ssm-data
		if( !::is_proc_shutoff() ){
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => create shared odometry error data with road surface environmental map named \"\x1b[4m%s\x1b[0m\"\n", pconf.odmerr_name.value );

			if( !ssm_odmerr.create(pconf.odmerr_name.value, pconf.odmerr_id.value, 5, 0.005) ){
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to open shared-data \"\x1b[4m%s\x1b[0m\"(id:%d)\n", pconf.odmerr_name.value, pconf.odmerr_id.value);
				::proc_shutoff();
			}
			else {
				::fprintf(stderr, " ... \x1b[1mOK\x1b[0m\n");
			}
		} // <--- create proofed odometry ssm-data


		// ---> open shared wheel encoder data
		if( !::is_proc_shutoff() ){
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => open shared wheel encoder data named \"\x1b[4m%s\x1b[0m\"\n", pconf.pos_name.value );

			if( !ssm_wenc.openWait(pconf.wenc_name.value, pconf.wenc_id.value, 0.0) ){
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to open shared-data \"\x1b[4m%s\x1b[0m\"(id:%d)\n", pconf.wenc_name.value, pconf.wenc_id.value);
				::proc_shutoff();
			}
			else {
				::fprintf(stderr, " ... \x1b[1mOK\x1b[0m\n");
				// blocking
				ssm_wenc.setBlocking(true);
				ssm_wenc.readLast();
			}
		} // <--- open shared wheel encoder data


		// ---> open position ssm-data
		if( !::is_proc_shutoff() ){
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => open shared position data named \"\x1b[4m%s\x1b[0m\"\n", pconf.pos_name.value );

			if( !ssm_pos.openWait(pconf.pos_name.value, pconf.pos_id.value, 0.0) ){
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to open shared-data \"\x1b[4m%s\x1b[0m\"(id:%d)\n", pconf.pos_name.value, pconf.pos_id.value);
				::proc_shutoff();
			}
			else {
				::fprintf(stderr, " ... \x1b[1mOK\x1b[0m\n");
				ssm_pos.readLast();
			}
		} // <--- open position ssm-data




	} // <--- initialize



	if( !::is_proc_shutoff() ){ // ---> operation
		gnd::Time::IntervalTimer timer_show;	// clock
		double show_running = 0;
		struct {
			double x;
			double y;
			double theta;
		} show_err;



		{ // ---> cui setting
			pcui.set_command( cui_cmd, sizeof(cui_cmd) / sizeof(cui_cmd[0]) );
		} // <--- cui setting


		::fprintf(stdout, "operation\n");
		// ---> operation loop
		while( !::is_proc_shutoff() ) {
			{ // ---> cui
				int cuival = 0;
				int cuito = 0;
				char cuiarg[512];
				// zero reset buffer
				::memset( cuiarg, 0, sizeof(cuiarg) );

				// ---> cui: operation
				if(	pcui.poll(&cuival, cuiarg, sizeof(cuiarg), cuito ) > 0 ) {
					if( timer_show.cycle() > 0 ) { // ---> not cui mode
						// quit show status mode and change to cui mode
						timer_show.end();
						::fprintf(stderr, "-------------------- cui mode --------------------\n");
					} // <--- not cui mode
					else { // ---> cui mode
						switch( cuival ) {
						// quit
						case 'Q': ::proc_shutoff();	break;
						// help;
						default:
						case '\0':
						case 'h': pcui.show(stderr, "   "); break;
						// show status mode
						case 's': timer_show.begin(CLOCK_REALTIME, CuiShowCycle, CuiShowCycle);
						}
					} // <--- cui mode
					// flush
					pcui.poll(&cuival, cuiarg, sizeof(cuiarg), cuito );
					// show command line
					::fprintf(stderr, "  > ");
				} // <--- cui: operation
			} // <--- cui



			// ---> show status
			if( timer_show.clock() > 0) {
				// clear display
				::fprintf(stderr, "\x1b[0;0H\x1b[2J");
				{ // ---> show status
					::fprintf(stderr, "-------------------- \x1b[1m\x1b[36m%s\x1b[39m\x1b[0m --------------------\n", OdometryCorrector::proc_name);
					::fprintf(stdout, " running distance  : %lf\n", show_running );
					::fprintf(stdout, "    odometry error : %lf %lf %lf\n",
							show_err.x, show_err.y, show_err.theta);
				} // <--- show status
			} // <--- show status


			if( ssm_wenc.readNext() ) { // ---> main operation
				// ---> proofing odometry with road surface environmental map
				// .... flow
				// 		*0. get current position estimation
				//		 1. compute running distance
				//		 2. get odometry error estimation with reference to road surface environmental map and current position estimation
				//		 3. output position estimation with proofed odometry

				double r = 0;									// running distance

				double err_ratio_x, err_ratio_y, err_ratio_theta;
				// 0. get current position estimation
				if( !ssm_pos.readLast() )	continue;


				{ // ---> 1. compute running distance
					double rrq;	// right wheel rotation quantity
					double lrq;	// left wheel rotation quantity

					rrq = (2.0 * M_PI * ( (double) ssm_wenc.data.counter2 ) ) / ( odm_knm.count_rev * odm_knm.gear );
					lrq = (2.0 * M_PI * ( (double) ssm_wenc.data.counter1 ) ) / ( odm_knm.count_rev * odm_knm.gear );

					r = ( ( rrq * odm_knm.radius_r ) + ( lrq * odm_knm.radius_l ) ) / 2;
				} // <--- 1. compute running distance


				{ // ---> 2. get odometry error estimation with reference to road surface environmental map and current position estimation
					gnd::odometry::correction::get( &map, ssm_pos.data.x, ssm_pos.data.y, ssm_pos.data.theta, &err_ratio_x, &err_ratio_y, &err_ratio_theta);
				} // <--- 2. get odometry error estimation with reference to road surface environmental map and current position estimation


				{ // ---> 3. output odometry error estimation
					ssm_odmerr.data.dx = err_ratio_x * r;
					ssm_odmerr.data.dy = err_ratio_y * r;
					ssm_odmerr.data.dtheta = err_ratio_theta * r;
				} // <--- 3. output odometry error estimation

				ssm_odmerr.write( ssm_pos.time );

			}// ---> main operation

		} // <--- operation loop


	} // <--- operation



	{ // ---> finalize
		// finalize ssm
		::endSSM();

		::fprintf(stderr, " ... Finish\n");
	} // <--- finalize

	return 0;
}
