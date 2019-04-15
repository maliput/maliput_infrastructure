stage('setup environment') {
   sh './index/ci/jenkins/setup_environment'
}
stage('build workspace') {
   sh './index/ci/jenkins/build_workspace'
}
stage('build') {
   sh './index/ci/jenkins/build'
}
stage('test') {
   sh './index/ci/jenkins/test'
}