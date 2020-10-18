package org.firstinspires.ftc.teamcode.robot2020.persistence;

import androidx.room.Database;
import androidx.room.RoomDatabase;

@Database(entities = {MovementEntity.class}, version = 1)
public abstract class AppDatabase extends RoomDatabase {

    public abstract MovementEntityDAO movementEntityDAO();
}
