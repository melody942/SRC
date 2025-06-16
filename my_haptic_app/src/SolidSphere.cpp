#include <QHHeadersGLUT.h> 

int main(int argc, char* argv[])
{
    // 1. 初始化显示窗口和力反馈设备
    QHGLUT* DisplayObject = new QHGLUT(argc, argv);
    DeviceSpace* Omni = new DeviceSpace;
    DisplayObject->tell(Omni);

    // 2. 创建一个球体并设置其物理属性
    Sphere* gMySphere = new Sphere(25.0, 30.0);
    gMySphere->setStiffness(1.0); // 设置为最硬
    gMySphere->setFriction(0.4, 0.5); // 设置摩擦力

    // 3. 设置球体的视觉属性并加入场景
    gMySphere->setTranslation(0.0, 0.0, 0.0);
    gMySphere->setShapeColor(0.9, 0.2, 0.2);
    DisplayObject->tell(gMySphere);

    // 4. 创建一个探针游标
    Cursor* OmniCursor = new Cursor;
    DisplayObject->tell(OmniCursor);

    // 5. 设置提示文字
    char* descriptionText = (char*)"A solid sphere, compiled in ROS!";
    Text* description = new Text(20.0, descriptionText, 0.1, 0.9);
    description->setShapeColor(1.0, 1.0, 1.0);
    DisplayObject->tell(description);

    // 6. 启动主循环
    qhStart();

    return 0;
}