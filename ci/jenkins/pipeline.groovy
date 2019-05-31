stage('setup') {
   sh './index/ci/jenkins/setup_workspace'
   sh './index/ci/jenkins/ws ./index/ci/jenkins/pull_sources'
   sh './index/ci/jenkins/ws ./index/ci/jenkins/install_deps'
}
stage('build') {
   sh './index/ci/jenkins/ws ./index/ci/jenkins/build'
}
stage('test') {
   sh './index/ci/jenkins/ws ./index/ci/jenkins/test'
}
