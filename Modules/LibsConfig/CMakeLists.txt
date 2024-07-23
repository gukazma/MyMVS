set_target_properties(DenseCloud
                       PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_property(TARGET DenseCloud PROPERTY CUDA_ARCHITECTURES 60 70 75 80)

target_link_libraries(DenseCloud PUBLIC opencv_ml opencv_dnn opencv_core opencv_highgui)