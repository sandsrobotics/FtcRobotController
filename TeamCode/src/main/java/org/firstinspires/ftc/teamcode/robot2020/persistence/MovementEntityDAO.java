package org.firstinspires.ftc.teamcode.robot2020.persistence;

import androidx.room.Dao;
import androidx.room.Insert;
import androidx.room.Query;

import java.util.List;

@Dao
public interface MovementEntityDAO {

    @Insert
    public void insertAll(MovementEntity ... entities);

    @Query("SELECT * FROM Movement")
    public List<MovementEntity> getAll();

    @Query("SELECT * FROM MOVEMENT WHERE name = :name ORDER BY ID")
    public List<MovementEntity> loadMovementByName(String name);
}
