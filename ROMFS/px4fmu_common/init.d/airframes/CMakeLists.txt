############################################################################
#
#   Copyright (c) 2018 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

px4_add_romfs_files(
	# [0-999] Reserved (historical)"

	# [1000, 1999] Simulation setups"
	1000_rc_fw_easystar.hil
	1001_rc_quad_x.hil
	1002_standard_vtol.hil
	1100_rc_quad_x_sih.hil

	# [2000, 2999] Standard planes"
	2100_standard_plane
	2105_maja
	2106_albatross
	2200_mini_talon

	2507_cloudship

	# [3000, 3999] Flying wing"
	3000_generic_wing
	3030_io_camflyer
	3031_phantom
	3032_skywalker_x5
	3033_wingwing
	3034_fx79
	3035_viper
	3036_pigeon
	3037_parrot_disco_mod
	3100_tbs_caipirinha

	# [4000, 4999] Quadrotor x"
	4001_quad_x
	4003_qavr5
	4009_qav250
	4010_dji_f330
	4011_dji_f450
	4014_s500
	4015_holybro_s500
	4016_holybro_px4vision
	4017_nxp_hovergames
	4018_s500_ctrlalloc
	4020_hk_micro_pcb
	4030_3dr_solo
	4031_3dr_quad
	4040_reaper
	4041_beta75x
	4050_generic_250
	4051_s250aq
	4052_holybro_qav250
	4053_holybro_kopis2
	4060_dji_matrice_100
	4071_ifo
	4072_draco
	4073_ifo-s
	4080_zmr250
	4090_nanomind
	4100_tiltquadrotor
	4250_teal
	4500_clover4
	4900_crazyflie
	4901_crazyflie21

	# [5000, 5999] Quadrotor +"
	5001_quad_+

	# [6000, 6999] Hexarotor x"
	6001_hexa_x
	6002_draco_r
	6003_hexa_x_ctrlalloc

	# [7000, 7999] Hexarotor +"
	7001_hexa_+

	# [8000, 8999] Octorotor +"
	8001_octo_x

	# [9000, 9999] Octorotor +"
	9001_octo_+

	# [10000, 10999] Quadrotor Wide arm / H frame"
	10015_tbs_discovery
	10016_3dr_iris
	10017_steadidrone_qu4d
	10018_tbs_endurance

	# [11000, 11999] Hexa Cox
	11001_hexa_cox

	# [12000, 12999] Octo Cox
	12001_octo_cox
	12002_steadidrone_mavrik

	# [13000, 13999] VTOL
	13000_generic_vtol_standard
	13001_caipirinha_vtol
	13002_firefly6
	13003_quad_tailsitter
	13004_quad+_tailsitter
	13005_vtol_AAERT_quad
	13006_vtol_standard_delta
	13007_vtol_AAVVT_quad
	13008_QuadRanger
	13009_vtol_spt_ranger
	13010_claire
	13012_convergence
	13013_deltaquad
	13014_vtol_babyshark
	13030_generic_vtol_quad_tiltrotor
	13050_generic_vtol_octo
	13200_generic_vtol_tailsitter

	# [14000, 14999] Tri Y
	14001_tri_y_yaw+
	14002_tri_y_yaw-

	15001_coax_heli

	16001_helicopter

	# [17000, 17999] Autogyro
	17002_TF-AutoG2
	17003_TF-G2

	# [18000, 18999] High-altitude balloons
	18001_TF-B1

	# [22000, 22999] Reserve for custom models

	24001_dodeca_cox

	50000_generic_ground_vehicle
	50003_aion_robotics_r1_rover
	50004_nxpcup_car_dfrobot_gpx

	# [60000, 61000] (Unmanned) Underwater Robots
	60000_uuv_generic
	60001_uuv_hippocampus
	60002_uuv_bluerov2_heavy
)
