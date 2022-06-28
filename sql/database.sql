-- MySQL Script generated by MySQL Workbench
-- T 28 juuni 2022 15:05:35
-- Model: New Model    Version: 1.0
-- MySQL Workbench Forward Engineering

SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0;
SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0;
SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='ONLY_FULL_GROUP_BY,STRICT_TRANS_TABLES,NO_ZERO_IN_DATE,NO_ZERO_DATE,ERROR_FOR_DIVISION_BY_ZERO,NO_ENGINE_SUBSTITUTION';

-- -----------------------------------------------------
-- Schema transport_ecosystem_management_db
-- -----------------------------------------------------

-- -----------------------------------------------------
-- Schema transport_ecosystem_management_db
-- -----------------------------------------------------
CREATE SCHEMA IF NOT EXISTS `transport_ecosystem_management_db` ;
USE `transport_ecosystem_management_db` ;

-- -----------------------------------------------------
-- Table `transport_ecosystem_management_db`.`dataset`
-- -----------------------------------------------------
CREATE TABLE IF NOT EXISTS `transport_ecosystem_management_db`.`dataset` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `name` VARCHAR(45) NOT NULL,
  `params` JSON NULL,
  PRIMARY KEY (`id`))
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `transport_ecosystem_management_db`.`ml_model`
-- -----------------------------------------------------
CREATE TABLE IF NOT EXISTS `transport_ecosystem_management_db`.`ml_model` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `name` VARCHAR(45) NOT NULL,
  `params` JSON NULL,
  PRIMARY KEY (`id`))
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `transport_ecosystem_management_db`.`detection_model`
-- -----------------------------------------------------
CREATE TABLE IF NOT EXISTS `transport_ecosystem_management_db`.`detection_model` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `ml_model_id` INT NOT NULL,
  `dataset_id` INT NOT NULL,
  `name` VARCHAR(45) NOT NULL,
  PRIMARY KEY (`id`),
  INDEX `fk_detection_model_dataset1_idx` (`dataset_id` ASC) VISIBLE,
  INDEX `fk_detection_model_ml_model1_idx` (`ml_model_id` ASC) VISIBLE,
  CONSTRAINT `fk_detection_model_dataset1`
    FOREIGN KEY (`dataset_id`)
    REFERENCES `transport_ecosystem_management_db`.`dataset` (`id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION,
  CONSTRAINT `fk_detection_model_ml_model1`
    FOREIGN KEY (`ml_model_id`)
    REFERENCES `transport_ecosystem_management_db`.`ml_model` (`id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `transport_ecosystem_management_db`.`object_class`
-- -----------------------------------------------------
CREATE TABLE IF NOT EXISTS `transport_ecosystem_management_db`.`object_class` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `name` VARCHAR(45) NOT NULL,
  PRIMARY KEY (`id`))
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `transport_ecosystem_management_db`.`frame`
-- -----------------------------------------------------
CREATE TABLE IF NOT EXISTS `transport_ecosystem_management_db`.`frame` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `timestamp` DATETIME NOT NULL,
  PRIMARY KEY (`id`))
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `transport_ecosystem_management_db`.`object`
-- -----------------------------------------------------
CREATE TABLE IF NOT EXISTS `transport_ecosystem_management_db`.`object` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `detection_model_id` INT NOT NULL,
  `class_id` INT NOT NULL,
  `frame_id` INT NOT NULL,
  `bounding_box` MULTIPOINT NOT NULL,
  `speed` FLOAT NULL,
  `distance` FLOAT NULL,
  `params` JSON NULL,
  `comment` VARCHAR(45) NULL,
  PRIMARY KEY (`id`),
  INDEX `fk_object_detection_model_idx` (`detection_model_id` ASC) VISIBLE,
  INDEX `fk_object_object_class1_idx` (`class_id` ASC) VISIBLE,
  INDEX `fk_object_frame1_idx` (`frame_id` ASC) VISIBLE,
  CONSTRAINT `fk_object_detection_model`
    FOREIGN KEY (`detection_model_id`)
    REFERENCES `transport_ecosystem_management_db`.`detection_model` (`id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION,
  CONSTRAINT `fk_object_object_class1`
    FOREIGN KEY (`class_id`)
    REFERENCES `transport_ecosystem_management_db`.`object_class` (`id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION,
  CONSTRAINT `fk_object_frame1`
    FOREIGN KEY (`frame_id`)
    REFERENCES `transport_ecosystem_management_db`.`frame` (`id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `transport_ecosystem_management_db`.`sensor_type`
-- -----------------------------------------------------
CREATE TABLE IF NOT EXISTS `transport_ecosystem_management_db`.`sensor_type` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `type_name` VARCHAR(45) NOT NULL,
  `params` JSON NULL,
  PRIMARY KEY (`id`))
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `transport_ecosystem_management_db`.`sensor`
-- -----------------------------------------------------
CREATE TABLE IF NOT EXISTS `transport_ecosystem_management_db`.`sensor` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `name` VARCHAR(45) NOT NULL,
  `params` JSON NULL,
  `sensor_type_id` INT NOT NULL,
  `comment` VARCHAR(45) NULL,
  `ros_topic` VARCHAR(45) NOT NULL,
  PRIMARY KEY (`id`),
  INDEX `fk_sensor_sensor_type1_idx` (`sensor_type_id` ASC) VISIBLE,
  CONSTRAINT `fk_sensor_sensor_type1`
    FOREIGN KEY (`sensor_type_id`)
    REFERENCES `transport_ecosystem_management_db`.`sensor_type` (`id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `transport_ecosystem_management_db`.`timestamps`
-- -----------------------------------------------------
CREATE TABLE IF NOT EXISTS `transport_ecosystem_management_db`.`timestamps` (
  `create_time` TIMESTAMP NULL DEFAULT CURRENT_TIMESTAMP,
  `update_time` TIMESTAMP NULL);


-- -----------------------------------------------------
-- Table `transport_ecosystem_management_db`.`user`
-- -----------------------------------------------------
CREATE TABLE IF NOT EXISTS `transport_ecosystem_management_db`.`user` (
  `username` VARCHAR(16) NOT NULL,
  `email` VARCHAR(255) NULL,
  `password` VARCHAR(32) NOT NULL,
  `create_time` TIMESTAMP NULL DEFAULT CURRENT_TIMESTAMP);


-- -----------------------------------------------------
-- Table `transport_ecosystem_management_db`.`category`
-- -----------------------------------------------------
CREATE TABLE IF NOT EXISTS `transport_ecosystem_management_db`.`category` (
  `category_id` INT NOT NULL,
  `name` VARCHAR(255) NOT NULL,
  PRIMARY KEY (`category_id`));


-- -----------------------------------------------------
-- Table `transport_ecosystem_management_db`.`frame_sensor`
-- -----------------------------------------------------
CREATE TABLE IF NOT EXISTS `transport_ecosystem_management_db`.`frame_sensor` (
  `frame_id` INT NOT NULL,
  `sensor_id` INT NOT NULL,
  PRIMARY KEY (`frame_id`, `sensor_id`),
  INDEX `fk_frame_sensor_sensor1_idx` (`sensor_id` ASC) VISIBLE,
  CONSTRAINT `fk_frame_sensor_frame1`
    FOREIGN KEY (`frame_id`)
    REFERENCES `transport_ecosystem_management_db`.`frame` (`id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION,
  CONSTRAINT `fk_frame_sensor_sensor1`
    FOREIGN KEY (`sensor_id`)
    REFERENCES `transport_ecosystem_management_db`.`sensor` (`id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


SET SQL_MODE=@OLD_SQL_MODE;
SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS;
SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS;
