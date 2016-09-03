node {
   stage 'Checkout'
   git 'https://github.com/jeschkies/renderer'
   stash name: 'scenes', includes: 'scenes/cornell_box.blend'

   stage 'Bootstrap'
   sh 'CXX=clang++ CC=clang make bootstrap'

   stage 'Build'
   sh 'CXX=clang++ CC=clang make -j4'
   stash name: 'binaries', includes: 'pathtracer,raytracer,raycaster,radiosity'

   stage 'Test'
   //unstash 'binaries'
   sh 'CXX=clang++ CC=clang make test'

   stage 'Render Test Images'
   //unstash 'scenes'
   sh 'mkdir -p rendered/$BUILD_NUMBER'

   // Render with raycaster
   sh './raycaster scenes/cornell_box.blend 640 > rendered/$BUILD_NUMBER/cornell_box_raycaster.pbm'
   sh 'convert rendered/$BUILD_NUMBER/cornell_box_raycaster.pbm rendered/$BUILD_NUMBER/cornell_box_raycaster.png'

   // Render with raytracer
   sh './raytracer scenes/cornell_box.blend -t4 > rendered/$BUILD_NUMBER/cornell_box_raytracer.pbm'
   sh 'convert rendered/$BUILD_NUMBER/cornell_box_raytracer.pbm rendered/$BUILD_NUMBER/cornell_box_raytracer.png'

   // Render with pathtracer
   sh './pathtracer scenes/cornell_box.blend -t4 -m4 -d2 > rendered/$BUILD_NUMBER/cornell_box_pathtracer.pbm'
   sh 'convert rendered/$BUILD_NUMBER/cornell_box_pathtracer.pbm rendered/$BUILD_NUMBER/cornell_box_pathtracer.png'

   // Render with radiosity
   sh './radiosity scenes/cornell_box.blend -t4 > rendered/$BUILD_NUMBER/cornell_box_radiosity.pbm'
   sh 'convert rendered/$BUILD_NUMBER/cornell_box_radiosity.pbm rendered/$BUILD_NUMBER/cornell_box_radiosity.png'

}
