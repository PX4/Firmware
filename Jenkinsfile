#!/usr/bin/env groovy

pipeline {
  agent none
  stages {

    stage('Analysis') {
      when {
        anyOf {
          branch 'master'
          branch 'pr-jenkins' // for testing
        }
      }
      parallel {
        // stage('Catkin build on ROS workspace') {
        //   agent {
        //     docker {
        //       image 'px4io/px4-dev-ros-melodic:2021-02-04'
        //       args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE'
        //     }
        //   }
        //   steps {
        //     sh 'ls -l'
        //     sh '''#!/bin/bash -l
        //       echo $0;
        //       mkdir -p catkin_ws/src;
        //       cd catkin_ws;
        //       git -C ${WORKSPACE}/catkin_ws/src/Firmware submodule update --init --recursive --force Tools/sitl_gazebo
        //       git clone --recursive ${WORKSPACE}/catkin_ws/src/Firmware/Tools/sitl_gazebo src/mavlink_sitl_gazebo;
        //       git -C ${WORKSPACE}/catkin_ws/src/Firmware fetch --tags;
        //       source /opt/ros/melodic/setup.bash;
        //       export PYTHONPATH=/opt/ros/$ROS_DISTRO/lib/python2.7/dist-packages:/usr/lib/python2.7/dist-packages:/usr/local/lib/python2.7/dist-packages;
        //       catkin init;
        //       catkin build -j$(nproc) -l$(nproc);
        //     '''
        //     // test if the binary was correctly installed and runs using 'mavros_posix_silt.launch'
        //     sh '''#!/bin/bash -l
        //       echo $0;
        //       source catkin_ws/devel/setup.bash;
        //       rostest px4 pub_test.launch;
        //     '''
        //   }
        //   post {
        //     always {
        //       sh 'rm -rf catkin_ws'
        //     }
        //     failure {
        //       archiveArtifacts(allowEmptyArchive: false, artifacts: '.ros/**/*.xml, .ros/**/*.log')
        //     }
        //   }
        //   options {
        //     checkoutToSubdirectory('catkin_ws/src/Firmware')
        //   }
        // }

        stage('Colcon build on ROS2 workspace') {
          agent {
            docker {
              image 'px4io/px4-dev-ros2-foxy:2021-02-04'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE'
            }
          }
          steps {
            sh 'ls -l'
            sh '''#!/bin/bash -l
              echo $0;
              unset ROS_DISTRO;
              mkdir -p colcon_ws/src;
              cd colcon_ws;
              git -C ${WORKSPACE}/colcon_ws/src/Firmware submodule update --init --recursive --force Tools/sitl_gazebo;
              git -C ${WORKSPACE}/colcon_ws/src/Firmware fetch --tags;
              source /opt/ros/foxy/setup.sh;
              colcon build --event-handlers console_direct+ --symlink-install;
            '''
          }
          post {
            always {
              sh 'rm -rf colcon_ws'
            }
          }
          options {
            checkoutToSubdirectory('colcon_ws/src/Firmware')
          }
        }

        stage('Airframe') {
          agent {
            docker { image 'px4io/px4-dev-base-focal:2021-02-04' }
          }
          steps {
            sh 'make distclean'
            sh 'git fetch --all --tags'
            sh 'make airframe_metadata'
            dir('build/px4_sitl_default/docs') {
              archiveArtifacts(artifacts: 'airframes.md, airframes.xml')
              stash includes: 'airframes.md, airframes.xml', name: 'metadata_airframes'
            }
          }
          post {
            always {
              sh 'make distclean'
            }
          }
        }

        stage('Parameter') {
          agent {
            docker { image 'px4io/px4-dev-base-focal:2021-02-04' }
          }
          steps {
            sh 'make distclean'
            sh 'git fetch --all --tags'
            sh 'make parameters_metadata'
            dir('build/px4_sitl_default/docs') {
              archiveArtifacts(artifacts: 'parameters.md, parameters.xml, parameters.json.xz')
              stash includes: 'parameters.md, parameters.xml, parameters.json.xz', name: 'metadata_parameters'
            }
          }
          post {
            always {
              sh 'make distclean'
            }
          }
        }

        stage('Module') {
          agent {
            docker { image 'px4io/px4-dev-base-focal:2021-02-04' }
          }
          steps {
            sh 'make distclean'
            sh 'git fetch --all --tags'
            sh 'make module_documentation'
            dir('build/px4_sitl_default/docs') {
              archiveArtifacts(artifacts: 'modules/*.md')
              stash includes: 'modules/*.md', name: 'metadata_module_documentation'
            }
          }
          post {
            always {
              sh 'make distclean'
            }
          }
        }

        stage('uORB graphs') {
          agent {
            docker {
              image 'px4io/px4-dev-nuttx-focal:2021-02-04'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'git fetch --all --tags'
            sh 'make uorb_graphs'
            dir('Tools/uorb_graph') {
              archiveArtifacts(artifacts: 'graph_px4_sitl.json')
              stash includes: 'graph_px4_sitl.json', name: 'uorb_graph'
            }
          }
          post {
            always {
              sh 'make distclean'
            }
          }
        }

      } // parallel
    } // stage: Generate Metadata

    stage('Deploy') {

      parallel {

        stage('Userguide') {
          agent {
            docker { image 'px4io/px4-dev-base-focal:2021-02-04' }
          }
          steps {
            sh('export')
            unstash 'metadata_airframes'
            unstash 'metadata_parameters'
            unstash 'metadata_module_documentation'
            withCredentials([usernamePassword(credentialsId: 'px4buildbot_github_personal_token', passwordVariable: 'GIT_PASS', usernameVariable: 'GIT_USER')]) {
              sh('git clone https://${GIT_USER}:${GIT_PASS}@github.com/PX4/px4_user_guide.git')
              sh('cp airframes.md px4_user_guide/en/airframes/airframe_reference.md')
              sh('cp parameters.md px4_user_guide/en/advanced_config/parameter_reference.md')
              sh('cp -R modules/*.md px4_user_guide/en/modules/')
              sh('cd px4_user_guide; git status; git add .; git commit -a -m "Update PX4 Firmware metadata `date`" || true')
              sh('cd px4_user_guide; git push origin master || true')
              sh('rm -rf px4_user_guide')
            }
          }
          when {
            anyOf {
              branch 'master'
              branch 'pr-jenkins' // for testing
            }
          }
          options {
            skipDefaultCheckout()
          }
        }

        stage('QGroundControl') {
          agent {
            docker { image 'px4io/px4-dev-base-focal:2021-02-04' }
          }
          steps {
            sh('export')
            unstash 'metadata_airframes'
            unstash 'metadata_parameters'
            withCredentials([usernamePassword(credentialsId: 'px4buildbot_github_personal_token', passwordVariable: 'GIT_PASS', usernameVariable: 'GIT_USER')]) {
              sh('git clone https://${GIT_USER}:${GIT_PASS}@github.com/mavlink/qgroundcontrol.git')
              sh('cp airframes.xml qgroundcontrol/src/AutoPilotPlugins/PX4/AirframeFactMetaData.xml')
              sh('cp parameters.xml qgroundcontrol/src/FirmwarePlugin/PX4/PX4ParameterFactMetaData.xml')
              sh('cd qgroundcontrol; git status; git add .; git commit -a -m "Update PX4 Firmware metadata `date`" || true')
              sh('cd qgroundcontrol; git push origin master || true')
              sh('rm -rf qgroundcontrol')
            }
          }
          when {
            anyOf {
              branch 'master'
              branch 'pr-jenkins' // for testing
            }
          }
          options {
            skipDefaultCheckout()
          }
        }

        stage('microRTPS agent') {
          agent {
            docker { image 'px4io/px4-dev-base-focal:2021-02-04' }
          }
          steps {
            sh('export')
            sh('git fetch --all --tags')
            sh('make distclean')
            sh('make px4_sitl_rtps')
            withCredentials([usernamePassword(credentialsId: 'px4buildbot_github_personal_token', passwordVariable: 'GIT_PASS', usernameVariable: 'GIT_USER')]) {
              sh("git clone https://${GIT_USER}:${GIT_PASS}@github.com/PX4/micrortps_agent.git -b ${BRANCH_NAME}")
              sh("rm -rf micrortps_agent/src micrortps_agent/idl")
              sh('cp -R build/px4_sitl_rtps/src/modules/micrortps_bridge/micrortps_agent/* micrortps_agent')
              sh('cd micrortps_agent; git status; git add src; git commit -a -m "Update microRTPS agent source code `date`" || true')
              sh('cd micrortps_agent; git push origin ${BRANCH_NAME} || true')
              sh('cd micrortps_agent; git status; git add idl; git commit -a -m "Update IDL definitions `date`" || true')
              sh('cd micrortps_agent; git push origin ${BRANCH_NAME} || true')
              sh('cd micrortps_agent; git status; git add CMakeLists.txt; git commit -a -m "Update CMakeLists.txt `date`" || true')
              sh('cd micrortps_agent; git push origin ${BRANCH_NAME} || true')
              sh('rm -rf micrortps_agent')
            }
          }
          when {
            anyOf {
              branch 'master'
              branch 'pr-jenkins' // for testing
            }
          }
        }

        stage('PX4 ROS msgs') {
          agent {
            docker { image 'px4io/px4-dev-base-focal:2021-02-04' }
          }
          steps {
            sh('export')
            sh('make distclean')
            withCredentials([usernamePassword(credentialsId: 'px4buildbot_github_personal_token', passwordVariable: 'GIT_PASS', usernameVariable: 'GIT_USER')]) {
              sh("git clone https://${GIT_USER}:${GIT_PASS}@github.com/PX4/px4_msgs.git")
              // 'master' branch
              sh('./msg/tools/uorb_to_ros_msgs.py msg/ px4_msgs/msg/')
              sh('cd px4_msgs; git status; git add .; git commit -a -m "Update message definitions `date`" || true')
              sh('cd px4_msgs; git push origin master || true')
              // 'ros1' branch
              sh('cd px4_msgs; git checkout ros1')
              sh('./msg/tools/uorb_to_ros_msgs.py msg/ px4_msgs/msg/')
              sh('cd px4_msgs; git status; git add .; git commit -a -m "Update message definitions `date`" || true')
              sh('cd px4_msgs; git push origin ros1 || true')
              sh('rm -rf px4_msgs')
            }
          }
          when {
            anyOf {
              branch 'master'
              branch 'pr-jenkins' // for testing
            }
          }
        }

        stage('PX4 ROS2 bridge') {
          agent {
            docker { image 'px4io/px4-dev-base-focal:2021-02-04' }
          }
          steps {
            sh('export')
            sh('make distclean')
            withCredentials([usernamePassword(credentialsId: 'px4buildbot_github_personal_token', passwordVariable: 'GIT_PASS', usernameVariable: 'GIT_USER')]) {
              sh("git clone https://${GIT_USER}:${GIT_PASS}@github.com/PX4/px4_ros_com.git -b ${BRANCH_NAME}")
              // deploy uORB RTPS ID map
              sh('./msg/tools/uorb_to_ros_rtps_ids.py -i msg/tools/uorb_rtps_message_ids.yaml -o px4_ros_com/templates/uorb_rtps_message_ids.yaml')
              sh('cd px4_ros_com; git status; git add .; git commit -a -m "Update uORB RTPS ID map `date`" || true')
              sh('cd px4_ros_com; git push origin ${BRANCH_NAME} || true')
              // deploy uORB RTPS required tools
              sh('cp msg/tools/uorb_rtps_classifier.py px4_ros_com/scripts/uorb_rtps_classifier.py')
              sh('cp msg/tools/generate_microRTPS_bridge.py px4_ros_com/scripts/generate_microRTPS_bridge.py')
              sh('cp msg/tools/px_generate_uorb_topic_files.py px4_ros_com/scripts/px_generate_uorb_topic_files.py')
              sh('cp msg/tools/px_generate_uorb_topic_helper.py px4_ros_com/scripts/px_generate_uorb_topic_helper.py')
              // deploy templates
              sh('cp msg/templates/urtps/microRTPS_agent.cpp.em px4_ros_com/templates/microRTPS_agent.cpp.em')
              sh('cp msg/templates/urtps/microRTPS_timesync.cpp.em px4_ros_com/templates/microRTPS_timesync.cpp.em')
              sh('cp msg/templates/urtps/microRTPS_timesync.h.em px4_ros_com/templates/microRTPS_timesync.h.em')
              sh('cp msg/templates/urtps/microRTPS_transport.cpp px4_ros_com/templates/microRTPS_transport.cpp')
              sh('cp msg/templates/urtps/microRTPS_transport.h px4_ros_com/templates/microRTPS_transport.h')
              sh('cp msg/templates/urtps/Publisher.cpp.em px4_ros_com/templates/Publisher.cpp.em')
              sh('cp msg/templates/urtps/Publisher.h.em px4_ros_com/templates/Publisher.h.em')
              sh('cp msg/templates/urtps/Subscriber.cpp.em px4_ros_com/templates/Subscriber.cpp.em')
              sh('cp msg/templates/urtps/Subscriber.h.em px4_ros_com/templates/Subscriber.h.em')
              sh('cp msg/templates/urtps/RtpsTopics.cpp.em px4_ros_com/templates/RtpsTopics.cpp.em')
              sh('cp msg/templates/urtps/RtpsTopics.h.em px4_ros_com/templates/RtpsTopics.h.em')
              sh('cd px4_ros_com; git status; git add .; git commit -a -m "Update uORB RTPS script tools `date`" || true')
              sh('cd px4_ros_com; git push origin ${BRANCH_NAME} || true')
              sh('rm -rf px4_msgs')
            }
          }
          when {
            anyOf {
              branch 'master'
              branch 'pr-jenkins' // for testing
            }
          }
        }

        stage('S3') {
          agent {
            docker { image 'px4io/px4-dev-base-focal:2021-02-04' }
          }
          steps {
            sh('export')
            unstash 'metadata_airframes'
            unstash 'metadata_parameters'
            sh('ls')
            withAWS(credentials: 'px4_aws_s3_key', region: 'us-east-1') {
              s3Upload(acl: 'PublicRead', bucket: 'px4-travis', file: 'airframes.xml', path: 'Firmware/master/')
              s3Upload(acl: 'PublicRead', bucket: 'px4-travis', file: 'parameters.xml', path: 'Firmware/master/')
              s3Upload(acl: 'PublicRead', bucket: 'px4-travis', file: 'parameters.json.xz', path: 'Firmware/master/')
            }
          }
          when {
            anyOf {
              branch 'master'
              branch 'pr-jenkins' // for testing
            }
          }
          options {
            skipDefaultCheckout()
          }
        }

      } // parallel
    } // stage: Generate Metadata

  } // stages

  environment {
    CCACHE_DIR = '/tmp/ccache'
    CI = true
    GIT_AUTHOR_EMAIL = "bot@px4.io"
    GIT_AUTHOR_NAME = "PX4BuildBot"
    GIT_COMMITTER_EMAIL = "bot@px4.io"
    GIT_COMMITTER_NAME = "PX4BuildBot"
  }
  options {
    buildDiscarder(logRotator(numToKeepStr: '10', artifactDaysToKeepStr: '20'))
    timeout(time: 60, unit: 'MINUTES')
  }
}
