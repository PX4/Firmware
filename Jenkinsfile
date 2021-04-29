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
