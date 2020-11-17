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
stage('deploy') {
  withCredentials([[$class: 'AmazonWebServicesCredentialsBinding',
                  credentialsId: 'c424c772-1817-4b13-9fe0-6ed2f22c0576',
                  accessKeyVariable: 'AWS_ACCESS_KEY_ID',
                  secretKeyVariable: 'AWS_SECRET_ACCESS_KEY']]) {
    sh './index/cd/jenkins/deploy'
  }
}
