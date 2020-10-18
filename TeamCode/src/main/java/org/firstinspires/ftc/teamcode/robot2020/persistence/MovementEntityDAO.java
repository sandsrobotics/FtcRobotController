package org.firstinspires.ftc.teamcode.robot2020.persistence;

import androidx.room.Dao;
import androidx.room.Insert;

@Dao
public interface MovementEntityDAO {

    @Insert
    void insertAll(MovementEntity ... entities);
}
