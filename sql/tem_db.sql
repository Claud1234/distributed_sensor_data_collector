--
-- PostgreSQL database dump
--

-- Dumped from database version 12.2
-- Dumped by pg_dump version 12.0

-- Started on 2022-03-28 15:16:10

SET statement_timeout = 0;
SET lock_timeout = 0;
SET idle_in_transaction_session_timeout = 0;
SET client_encoding = 'UTF8';
SET standard_conforming_strings = on;
SELECT pg_catalog.set_config('search_path', '', false);
SET check_function_bodies = false;
SET xmloption = content;
SET client_min_messages = warning;
SET row_security = off;

SET default_tablespace = '';

SET default_table_access_method = heap;

--
-- TOC entry 203 (class 1259 OID 459142)
-- Name: Detected_Objects; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public."Detected_Objects" (
    "Id" serial NOT NULL,
    "Class" text NOT NULL,
    "Speed" double precision,
    "Distance" double precision,
    "Bounding_Box" box NOT NULL,
    frame_id bigint
);



--
-- TOC entry 202 (class 1259 OID 459134)
-- Name: Frame; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public."Frame" (
    "Id" serial NOT NULL,
    "Sensor_Data" bigint NOT NULL,
    "Self_Speed" double precision NOT NULL,
    "GPS_Coords" double precision[2] NOT NULL,
    "Timestamp" bigint NOT NULL,
    "Processed" boolean DEFAULT false NOT NULL
);



--
-- TOC entry 204 (class 1259 OID 459150)
-- Name: Sensor_Data; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public."Sensor_Data" (
    "Id" serial NOT NULL,
    image_file text NOT NULL,
    radar_point_cloud text NOT NULL
);


--
-- TOC entry 2700 (class 2606 OID 459149)
-- Name: Detected_Objects DetectedObjects_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public."Detected_Objects"
    ADD CONSTRAINT "DetectedObjects_pkey" PRIMARY KEY ("Id");


--
-- TOC entry 2698 (class 2606 OID 459141)
-- Name: Frame Frame_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public."Frame"
    ADD CONSTRAINT "Frame_pkey" PRIMARY KEY ("Id");


--
-- TOC entry 2702 (class 2606 OID 459157)
-- Name: Sensor_Data SensorData_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public."Sensor_Data"
    ADD CONSTRAINT "SensorData_pkey" PRIMARY KEY ("Id");


--
-- TOC entry 2703 (class 2606 OID 459177)
-- Name: Frame SensorData_Id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public."Frame"
    ADD CONSTRAINT "SensorData_Id_fkey" FOREIGN KEY ("Sensor_Data") REFERENCES public."Sensor_Data"("Id") NOT VALID;


--
-- TOC entry 2704 (class 2606 OID 459182)
-- Name: Detected_Objects frame_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public."Detected_Objects"
    ADD CONSTRAINT frame_id_fkey FOREIGN KEY (frame_id) REFERENCES public."Frame"("Id") NOT VALID;


-- Completed on 2022-03-28 15:16:11

--
-- PostgreSQL database dump complete
--
