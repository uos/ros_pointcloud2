use ros_pointcloud2::PointConvertible;
use rpcl2_derive::*;

#[derive(Debug, PartialEq, Clone, Default, Copy, PointConvertible)]
#[repr(C, align(4))]
struct MyPointXYZI {
    x: f32,
    #[ros(remap("test"))]
    y: u16,
    z: f32,
    #[ros(remap("i"))]
    intensity: i32,
    label: u8,
}

#[test]
fn layout() {
    let layout_str = format!("{:?}", MyPointXYZI::layout());
    assert_eq!("LayoutDescription([Field { name: \"x\", ty: \"f32\", size: 4 }, Field { name: \"test\", ty: \"u16\", size: 2 }, Padding { size: 2 }, Field { name: \"z\", ty: \"f32\", size: 4 }, Field { name: \"i\", ty: \"i32\", size: 4 }, Field { name: \"label\", ty: \"u8\", size: 1 }, Padding { size: 3 }])", layout_str);
}

#[derive(Debug, PartialEq, Clone, Default, Copy, PointConvertible)]
#[repr(C)]
struct MixedWidthPoint {
    x: f32,
    y: f32,
    z: f32,
    intensity: f32,
    timestamp: f64,
}

#[test]
fn mixed_width_point_survives_a_cloud_round_trip() {
    let point = MixedWidthPoint {
        x: 1.0,
        y: 2.0,
        z: 3.0,
        intensity: 9.0,
        timestamp: 1_700_000_000.25,
    };
    let cloud = ros_pointcloud2::PointCloud2Msg::try_from_vec(vec![point]).unwrap();

    assert_eq!(cloud.point_step as usize, core::mem::size_of::<MixedWidthPoint>());
    let offsets: Vec<u32> = cloud.fields.iter().map(|field| field.offset).collect();
    assert_eq!(offsets, vec![0, 4, 8, 12, 16]);

    let back: Vec<MixedWidthPoint> = cloud.try_into_iter().unwrap().collect();
    assert_eq!(back, vec![point]);
}
