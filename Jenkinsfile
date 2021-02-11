node {
    stage('Fetch') {
        dir('modules') {
            checkout scm
            sh 'git submodule sync --recursive'
            sh 'git submodule update --init --recursive'
        }
        dir('inviwo') {
            git([url: 'https://github.com/inviwo/inviwo.git', branches: '*/master'])
            sh 'git submodule sync --recursive'
            sh 'git submodule update --init --recursive'
        }
    }

    def util = load "${env.WORKSPACE}/inviwo/tools/jenkins/util.groovy"
    util.config(this)

    def modulePaths = ["${env.WORKSPACE}/modules/misc",
                       "${env.WORKSPACE}/modules/molvis",
                       "${env.WORKSPACE}/modules/tensorvis",
                       "${env.WORKSPACE}/modules/topovis",
                       "${env.WORKSPACE}/modules/vectorvis"]
    util.wrap(this, "#jenkins-branch-pr") {
        util.format(this, "${env.WORKSPACE}/modules")
        util.buildStandard(
            state: this,
            modulePaths: modulePaths,
            onModules: ["TOPOLOGYTOOLKIT", "OPENMESH", "SPRINGSYSTEM", "VASP",
                        "NANOVGUTILS", "TENSORVISBASE", "INTEGRALLINEFILTERING",
                        "MOLVISBASE", "MOLVISGL", "MOLVISPYTHON",
                        "DATAFRAMECLUSTERING"],
            offModules: ["ABUFFERGL", "VTK", "TENSORVISIO"],
            opts: [:]
        )
        util.warn(this, 'daily/modules/appleclang')
        util.unittest(this)
        util.integrationtest(this)
        util.regression(this, modulePaths)
        util.copyright(this)
        util.doxygen(this)
    }
}
