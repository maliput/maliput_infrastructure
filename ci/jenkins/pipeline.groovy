stage('setup early') {
   sh './index/ci/jenkins/setup_early'
}
stage('build workspace') {
   sh './index/ci/jenkins/build_workspace'
}
stage('setup late') {
   sh './index/ci/jenkins/setup_late'
}
stage('build') {
   sh './index/ci/jenkins/build'
}
stage('test') {
   sh './index/ci/jenkins/test'
}