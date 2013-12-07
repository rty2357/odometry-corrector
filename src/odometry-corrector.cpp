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

#include "gnd-timer.hpp"
#include "gnd-odometry-correction.hpp"
#include "gnd-shutoff.hpp"

int main(int argc, char* argv[], char *env[]) {
	SSMApi<PWSMotor>					ssm_wenc;			// shared wheel encoder data (read)
	SSMApi<Spur_Odometry>				ssm_pos;			// shared position data (read)
	SSMApi_OdometryErr					ssm_odmerr;			// shared odometry error data (write)

	gnd::odometry::cmap					map;				// road surface environment map

	struct {
		double radius_r;
		double radius_l;
		double tread;
		double count_rev;
		double gear;
	} odm_knm;												// odometry kinematics parameter

	OdometryCorrector::proc_configuration	pconf;			// process configuration
	OdometryCorrector::proc_option_reader	popt(&pconf);	// process option reader
	gnd::cui_reader							pcui;			// cui



	{ // ---> initialize
		int ret;
		uint32_t phase = 1;


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
		double err_ratio_x = 0, err_ratio_y = 0, err_ratio_theta = 0;

		gnd::timer::interval_timer timer_show;	// clock
		double show_running = 0;
		int nline_show = 0;
		struct {
			double x;
			double y;
			double theta;
		} show_err, show_sum_err;
		double sum_r;

		show_err.x = show_err.y = show_err.theta = 0;
		show_sum_err.x = show_sum_err.y = show_sum_err.theta = 0;


		{ // ---> cui setting
			pcui.set_command( OdometryCorrector::cui_cmd, sizeof(OdometryCorrector::cui_cmd) / sizeof(OdometryCorrector::cui_cmd[0]) );
			timer_show.begin( CLOCK_REALTIME, OdometryCorrector::CuiShowCycle, -OdometryCorrector::CuiShowCycle );
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
						case 's': timer_show.begin(CLOCK_REALTIME, OdometryCorrector::CuiShowCycle, OdometryCorrector::CuiShowCycle); break;
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
				// back cursor
				if( nline_show ) {
					::fprintf(stderr, "\x1b[%02dA", nline_show);
					nline_show = 0;
				}
				{ // ---> show status
					nline_show++; ::fprintf(stderr, "\x1b[K-------------------- \x1b[1m\x1b[36m%s\x1b[39m\x1b[0m --------------------\n", OdometryCorrector::proc_name);
					nline_show++; ::fprintf(stdout, "\x1b[K running distance  : %lf\n", show_running );
					nline_show++; ::fprintf(stdout, "\x1b[K    odometry error : %lf %lf %lf\n", show_err.x, show_err.y, gnd_ang2deg(show_err.theta));
					nline_show++; ::fprintf(stdout, "\x1b[K         sum error : %lf %lf %lf, %lf\n", show_sum_err.x, show_sum_err.y, gnd_ang2deg(show_sum_err.theta), sum_r);
					nline_show++; ::fprintf(stdout, "\x1b[K       error ratio : %lf %lf %lf\n", err_ratio_x, err_ratio_y, gnd_ang2deg(err_ratio_theta) );
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

				// 0. get current position estimation
				if( !ssm_pos.readLast() )	continue;


				{ // ---> 1. compute running distance
					int cnt1 = (pconf.k_lwheel_crot.value ? -1 : 1) * ssm_wenc.data.counter1;
					int cnt2 = (pconf.k_rwheel_crot.value ? -1 : 1) * ssm_wenc.data.counter2;

					double rrq;	// right wheel rotation quantity
					double lrq;	// left wheel rotation quantity

					if( pconf.k_swap_rwmotor.value ){
						int swap = cnt1;
						cnt1 = cnt2;
						cnt2 = swap;
					}

					rrq = (2.0 * M_PI * ( (double) cnt2 ) ) / ( odm_knm.count_rev * odm_knm.gear );
					lrq = (2.0 * M_PI * ( (double) cnt1 ) ) / ( odm_knm.count_rev * odm_knm.gear );

					r = ( ( rrq * odm_knm.radius_r ) + ( lrq * odm_knm.radius_l ) ) / 2;
					show_running = r;
					sum_r +=r;
				} // <--- 1. compute running distance


				{ // ---> 2. get odometry error estimation with reference to road surface environmental map and current position estimation
					gnd::odometry::correction::get( &map, ssm_pos.data.x, ssm_pos.data.y, ssm_pos.data.theta, &err_ratio_x, &err_ratio_y, &err_ratio_theta);
				} // <--- 2. get odometry error estimation with reference to road surface environmental map and current position estimation


				if( r > 0 ){ // ---> 3. output odometry error estimation
					ssm_odmerr.data.dx = err_ratio_x * r;
					show_err.x = ssm_odmerr.data.dx;
					show_sum_err.x -= ssm_odmerr.data.dx;

					ssm_odmerr.data.dy = err_ratio_y * r;
					show_err.y = ssm_odmerr.data.dy;
					show_sum_err.y -= ssm_odmerr.data.dy;

					ssm_odmerr.data.dtheta = err_ratio_theta * r;
					show_err.theta = ssm_odmerr.data.dtheta;
					show_sum_err.theta -= ssm_odmerr.data.dtheta;
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
