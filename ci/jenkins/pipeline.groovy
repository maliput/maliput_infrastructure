stage('setup workspace') {
   sh './index/ci/jenkins/setup_ws'
}
stage('build workspace') {
   sh './index/ci/jenkins/ws ./index/ci/jenkins/build_ws'
}
stage('build') {
   sh './index/ci/jenkins/ws ./index/ci/jenkins/build'
}
stage('test') {
   sh './index/ci/jenkins/ws ./index/ci/jenkins/test'
}
