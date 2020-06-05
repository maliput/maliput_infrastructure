stage('setup') {
   sh './index/ci/jenkins/setup_workspace'
   sh './index/ci/jenkins/ws ./index/ci/jenkins/pull_sources'
   sh './index/ci/jenkins/ws ./index/ci/jenkins/install_deps'
}
stage('scan_build') {
   sh './index/ci/jenkins/ws ./index/ci/jenkins/scan_build'
}
