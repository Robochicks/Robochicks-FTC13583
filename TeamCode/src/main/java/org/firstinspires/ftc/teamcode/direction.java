package org.firstinspires.ftc.teamcode;

public enum direction {
        FORWARD(1, 1, 1, 1),
        BACKWARD(-1, -1, -1, -1),
        SHIFT_L(-1, 1, 1, -1),
        SHIFT_R(1, -1, -1, 1),
        TURN_L(-1, 1, -1, 1),
        TURN_R(1, -1, 1, -1);

        public final int fl_dir,
                fr_dir,
                bl_dir,
                br_dir;

         direction(int fl_dir, int fr_dir, int bl_dir, int br_dir) {
             this.fl_dir = fl_dir;
             this.fr_dir = fr_dir;
             this.bl_dir = bl_dir;
             this.br_dir = br_dir;
         }
}
