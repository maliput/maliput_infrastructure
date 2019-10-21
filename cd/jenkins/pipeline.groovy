withEnv(['COLCON_BUILD_EXTRA_ARGS=${env.COLCON_BUILD_EXTRA_ARGS} --cmake-args -DBUILD_DOCS=1']) {
  load './index/ci/jenkins/pipeline.groovy'
}
stage('deploy') {
  sh './index/cd/jenkins/publish_docs'
  withCredentials([[$class: 'AmazonWebServicesCredentialsBinding',
                  credentialsId: 'c424c772-1817-4b13-9fe0-6ed2f22c0576',
                  accessKeyVariable: 'AWS_ACCESS_KEY_ID',
                  secretKeyVariable: 'AWS_SECRET_ACCESS_KEY']]) {
    sh './index/cd/jenkins/deploy_binaries'
  }
}
