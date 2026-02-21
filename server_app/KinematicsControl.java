package lbr_fri_ros2;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.Arrays;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.motionModel.controlModeModel.*;
import com.kuka.connectivity.fastRobotInterface.*;

public class KinematicsControl extends RoboticsAPIApplication {
  // members
    @Inject
    private LBR lbr_;
    private Controller lbr_controller_;

    @Inject
    @Named("Tool")
    private Tool mytool;

  // control mode
    private enum CONTROL_MODE {
        POSITION_CONTROL,
        JOINT_IMPEDANCE_CONTROL,
        CARTESIAN_IMPEDANCE_CONTROL;
    }

  // convert enum to string array, see https://stackoverflow.com/questions/13783295/getting-all-names-in-an-enum-as-a-string
    public static String[] getNames(Class<? extends Enum<?>> e) {
        return Arrays.toString(e.getEnumConstants()).replaceAll("^.|.$", "").split(", ");
    }

  // FRI parameters
    private String client_name_;
    private String[] client_names_ = {"172.31.1.10", "10.66.171.34"};
    private int client_port_;
    private String[] client_ports_ = {"30200", "30201", "30202", "30203", "30204", "30205"};
    private int send_period_;
    private String[] send_periods_ = {"1", "2", "5", "10"};  // send period in ms

    private FRIConfiguration fri_configuration_;
    private FRISession fri_session_;
    private FRIJointOverlay fri_overlay_;

    private AbstractMotionControlMode control_mode_;
    private ClientCommandMode command_mode_;
  // methods
    public void request_user_config() {
      // send period
        int id = getApplicationUI().displayModalDialog(
            ApplicationDialogType.QUESTION,
            "Select the desired FRI send period [ms]:",
            send_periods_);
        send_period_ = Integer.valueOf(send_periods_[id]);
        getLogger().info("Send period set to: " + send_period_);

      // remote IP address
        id = getApplicationUI().displayModalDialog(
            ApplicationDialogType.QUESTION,
            "Select your remote IP address:",
            client_names_);
        client_name_ = client_names_[id];
        getLogger().info("Remote address set to: " + client_name_);

      // remote port
        id = getApplicationUI().displayModalDialog(
            ApplicationDialogType.QUESTION,
            "Select your remote port:",
            client_ports_);
        client_port_ = Integer.valueOf(client_ports_[id]);
        getLogger().info("Remote port set to: " + client_port_);

        control_mode_= 	new PositionControlMode();
        command_mode_ = ClientCommandMode.POSITION;
        getLogger().info("Robot is controlled with: POSITION CONTROL");
        getLogger().info("Client command mode set to: " + command_mode_.name());
    }

    public void configure_fri() {
        fri_configuration_ = FRIConfiguration.createRemoteConfiguration(lbr_, client_name_);
        fri_configuration_.setPortOnRemote(client_port_);
        fri_configuration_.setSendPeriodMilliSec(send_period_);
        getLogger().info("Creating FRI connection to " + fri_configuration_.getHostName());
        getLogger().info(
            "SendPeriod: " + fri_configuration_.getSendPeriodMilliSec() + "ms |"
            + " ReceiveMultiplier: " + fri_configuration_.getReceiveMultiplier());
        fri_session_ = new FRISession(fri_configuration_);
        fri_overlay_ = new FRIJointOverlay(fri_session_, command_mode_);
        fri_session_.addFRISessionListener(new IFRISessionListener() {
                @Override
                public void onFRISessionStateChanged(FRIChannelInformation friChannelInformation) {
                    getLogger().info("Session State change " + friChannelInformation.getFRISessionState().toString());
                }

                @Override
                public void onFRIConnectionQualityChanged(FRIChannelInformation friChannelInformation) {
                    getLogger().info("Quality change signalled " + friChannelInformation.getQuality());
                    getLogger().info("Jitter " + friChannelInformation.getJitter());
                    getLogger().info("Latency " + friChannelInformation.getLatency());
                }
            });
      // try to connect
        try {
            fri_session_.await(60, TimeUnit.SECONDS);
        } catch (final TimeoutException e) {
            getLogger().error(e.getLocalizedMessage());
            getLogger().error("Connection timeout: Current Timeout limit = 60 sec");
            return;
        }
        getLogger().info("FRI connection established.");
    }

    @Override
    public void initialize() {
        ObjectFrame lbr_flange = lbr_.getFlange();
        mytool.attachTo(lbr_flange);
        getLogger().info("End Effector position:" + lbr_flange.getX()/1000.0 + " " + lbr_flange.getY()/1000.0 + " " + lbr_flange.getZ()/1000.0);

        lbr_controller_ = (Controller) getContext().getControllers().toArray()[0];
        lbr_ = (LBR) lbr_controller_.getDevices().toArray()[0];

      // set FRI parameters
        request_user_config();
      // configure the FRI
        configure_fri();
    }

    @Override
    public void run() {
      // run the FRI
        lbr_.move(positionHold(control_mode_, -1, null).addMotionOverlay(fri_overlay_));
        return;
    }

    @Override
    public void dispose() {
      // close connection
        getLogger().info("Disposing FRI session.");
        fri_session_.close();

        super.dispose();
    }

  /**
   * main
   *
   * @param args
   */
    public static void main(final String[] args) {
        KinematicsControl app = new KinematicsControl();
        app.runApplication();
    }
}
