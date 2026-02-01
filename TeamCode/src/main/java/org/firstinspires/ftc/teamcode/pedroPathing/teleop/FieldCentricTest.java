package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "空间感知驾驶测试 (Field Centric)", group = "Test")
public class FieldCentricTest extends LinearOpMode {

    // 定义底盘马达
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    // 定义陀螺仪 (这是空间感知的核心)
    private IMU imu;

    @Override
    public void runOpMode() {
        // 1. 硬件映射 (名字要和你手机上的一样)
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        
        imu = hardwareMap.get(IMU.class, "imu");

        // 2. 设置马达方向 (根据你的 drive.java 配置)
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        // 右边通常是 FORWARD，如果发现转反了改这里
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        // 设置刹车模式 (松手即停，手感更好)
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 3. 初始化 IMU (陀螺仪) - 这一步最关键！
        // 请检查你的 Control Hub 是怎么安装的：
        // 下面的设置假设：Hub 平躺 (Logo UP)，接口朝前 (USB FORWARD)
        // 如果你的 Hub 是竖着装的，需要修改这里！
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        telemetry.addLine("初始化完成！");
        telemetry.addLine("操作说明：");
        telemetry.addLine("1. 左摇杆控制移动 (绝对方向)");
        telemetry.addLine("2. 右摇杆控制旋转");
        telemetry.addLine("3. 如果方向歪了，按【Options/Start】键重置");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // === 核心逻辑开始 ===

            // 1. 获取摇杆输入
            double y = -gamepad1.left_stick_y; // 前后 (推杆是负数，所以要取反)
            double x = gamepad1.left_stick_x;  // 左右
            double rx = gamepad1.right_stick_x; // 旋转

            // 2. 重置功能：如果驾驶员觉得"正前方"不对了，按这个键重置
            if (gamepad1.options) {
                imu.resetYaw();
            }

            // 3. 获取机器人当前的朝向 (弧度制)
            double botHeading = imu.getRobotYawPitchRoll().yaw;

            // 4. 空间感知数学公式 (旋转坐标系)
            // 这就是让机器人"忘了车头，只认场地"的魔法
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // 修正侧向移动速度 (麦克纳姆轮横移通常比直行慢，乘以1.1补偿)
            rotX = rotX * 1.1;

            // 5. 计算四个轮子的动力
            // 现在的 rotY 和 rotX 已经是相对于场地的了
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // 6. 输出动力
            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            // === 遥测显示 ===
            telemetry.addData("模式", "Field Centric (空间感知)");
            telemetry.addData("车头朝向", "%.1f 度", Math.toDegrees(botHeading));
            telemetry.update();
        }
    }
}