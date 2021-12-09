add_library(pca_main
        CoordinateTrans.cpp
        MonoCameraTrans.cpp
        NJUST_Data.cpp
        NJUST_Serialize.cpp
        NJUST_Tcp.cpp)
target_link_libraries(pca_main
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        pcap)

add_library(pca_debug
        NJUST_Lidar32.cpp
)
target_link_libraries(pca_debug
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES})

add_library(pca_obs
        NJUST_Lidar32.cpp
        Lidar32OD.cpp)
target_link_libraries(pca_obs
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES})

add_library(pca_trans
        CoordinateTrans.cpp
        MonoCameraTrans.cpp
        NJUST_Transform.cpp)

add_library(pca_fusion
        NJUST_Lidar32.cpp
        NJUST_Fusion.cpp)
target_link_libraries(pca_fusion
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES})


add_library(pca_location
        NJUST_Location.cpp)
target_link_libraries(pca_location
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES})




