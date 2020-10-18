package org.firstinspires.ftc.teamcode.robot2020.persistence;

import androidx.room.ColumnInfo;
import androidx.room.Entity;
import androidx.room.PrimaryKey;

@Entity(tableName = "Movement")
public class MovementEntity {

    @PrimaryKey (autoGenerate = true)
    public int id;

    @ColumnInfo(name = "name")
    public String name;
    @ColumnInfo(name = "motor_id")
    public int motor_id;
    @ColumnInfo(name = "motor_tick")
    public int motor_tick;
    @ColumnInfo(name = "motor_velocity")
    public double motor_velocity;

    public MovementEntity() {
    }
    public MovementEntity(String name, int motor_id, int motor_tick, double motor_velocity) {
        this.name = name;
        this.motor_id = motor_id;
        this.motor_tick = motor_tick;
        this.motor_velocity = motor_velocity;
    }
}
