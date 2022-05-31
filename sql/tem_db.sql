--
-- PostgreSQL database dump
--

-- Dumped from database version 14.3
-- Dumped by pg_dump version 14.3

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
-- Name: DetectedObject; Type: TABLE; Schema: public; Owner: db_user
--

CREATE TABLE public."DetectedObject" (
    "ID" integer NOT NULL,
    "Class" text NOT NULL,
    "Speed" double precision,
    "Distance" double precision,
    "BoundingBox" box NOT NULL,
    "FrameID" bigint NOT NULL,
    "ModelID" bigint NOT NULL
);


ALTER TABLE public."DetectedObject" OWNER TO db_user;

--
-- Name: DetectedObject_ID_seq; Type: SEQUENCE; Schema: public; Owner: db_user
--

CREATE SEQUENCE public."DetectedObject_ID_seq"
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER TABLE public."DetectedObject_ID_seq" OWNER TO db_user;

--
-- Name: DetectedObject_ID_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: db_user
--

ALTER SEQUENCE public."DetectedObject_ID_seq" OWNED BY public."DetectedObject"."ID";


--
-- Name: Model; Type: TABLE; Schema: public; Owner: db_user
--

CREATE TABLE public."Model" (
    "ID" integer NOT NULL,
    "ModelName" text NOT NULL,
    "DatasetName" text NOT NULL,
    "InputSize" integer[] NOT NULL
);


ALTER TABLE public."Model" OWNER TO db_user;

--
-- Name: Model_ID_seq; Type: SEQUENCE; Schema: public; Owner: db_user
--

CREATE SEQUENCE public."Model_ID_seq"
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER TABLE public."Model_ID_seq" OWNER TO db_user;

--
-- Name: Model_ID_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: db_user
--

ALTER SEQUENCE public."Model_ID_seq" OWNED BY public."Model"."ID";


--
-- Name: Frame; Type: TABLE; Schema: public; Owner: db_user
--

CREATE TABLE public."Frame" (
    "ID" integer NOT NULL,
    "SelfSpeed" double precision NOT NULL,
    "GpsCoords" double precision[] NOT NULL,
    "Timestamp" bigint NOT NULL,
    "Processed" boolean DEFAULT false NOT NULL
);


ALTER TABLE public."Frame" OWNER TO db_user;

--
-- Name: FrameID_seq; Type: SEQUENCE; Schema: public; Owner: db_user
--

CREATE SEQUENCE public."FrameID_seq"
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER TABLE public."FrameID_seq" OWNER TO db_user;

--
-- Name: FrameID_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: db_user
--

ALTER SEQUENCE public."FrameID_seq" OWNED BY public."Frame"."ID";


--
-- Name: SensorData; Type: TABLE; Schema: public; Owner: db_user
--

CREATE TABLE public."SensorData" (
    "ID" integer NOT NULL,
    "SensorType" text NOT NULL,
    "DataFile" text NOT NULL,
    "FrameID" bigint NOT NULL
);


ALTER TABLE public."SensorData" OWNER TO db_user;

--
-- Name: SensorData_ID_seq; Type: SEQUENCE; Schema: public; Owner: db_user
--

CREATE SEQUENCE public."SensorData_ID_seq"
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER TABLE public."SensorData_ID_seq" OWNER TO db_user;

--
-- Name: SensorData_ID_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: db_user
--

ALTER SEQUENCE public."SensorData_ID_seq" OWNED BY public."SensorData"."ID";


--
-- Name: DetectedObject ID; Type: DEFAULT; Schema: public; Owner: db_user
--

ALTER TABLE ONLY public."DetectedObject" ALTER COLUMN "ID" SET DEFAULT nextval('public."DetectedObject_ID_seq"'::regclass);


--
-- Name: Model ID; Type: DEFAULT; Schema: public; Owner: db_user
--

ALTER TABLE ONLY public."Model" ALTER COLUMN "ID" SET DEFAULT nextval('public."Model_ID_seq"'::regclass);


--
-- Name: Frame ID; Type: DEFAULT; Schema: public; Owner: db_user
--

ALTER TABLE ONLY public."Frame" ALTER COLUMN "ID" SET DEFAULT nextval('public."FrameID_seq"'::regclass);


--
-- Name: SensorData ID; Type: DEFAULT; Schema: public; Owner: db_user
--

ALTER TABLE ONLY public."SensorData" ALTER COLUMN "ID" SET DEFAULT nextval('public."SensorData_ID_seq"'::regclass);


--
-- Data for Name: DetectedObject; Type: TABLE DATA; Schema: public; Owner: db_user
--

COPY public."DetectedObject" ("ID", "Class", "Speed", "Distance", "BoundingBox", FrameID, "ModelID") FROM stdin;
\.


--
-- Data for Name: Model; Type: TABLE DATA; Schema: public; Owner: db_user
--

COPY public."Model" ("ID", "ModelName", "DatasetName", "InputSize") FROM stdin;
\.


--
-- Data for Name: Frame; Type: TABLE DATA; Schema: public; Owner: db_user
--

COPY public."Frame" ("ID", "SelfSpeed", "GpsCoords", "Timestamp", "Processed") FROM stdin;
\.


--
-- Data for Name: SensorData; Type: TABLE DATA; Schema: public; Owner: db_user
--

COPY public."SensorData" ("ID", "SensorType", "DataFile", "FrameID") FROM stdin;
\.


--
-- Name: DetectedObject_ID_seq; Type: SEQUENCE SET; Schema: public; Owner: db_user
--

SELECT pg_catalog.setval('public."DetectedObject_ID_seq"', 1, false);


--
-- Name: Model_ID_seq; Type: SEQUENCE SET; Schema: public; Owner: db_user
--

SELECT pg_catalog.setval('public."Model_ID_seq"', 1, false);


--
-- Name: FrameID_seq; Type: SEQUENCE SET; Schema: public; Owner: db_user
--

SELECT pg_catalog.setval('public."FrameID_seq"', 1, false);


--
-- Name: SensorData_ID_seq; Type: SEQUENCE SET; Schema: public; Owner: db_user
--

SELECT pg_catalog.setval('public."SensorData_ID_seq"', 1, false);


--
-- Name: DetectedObject DetectedObject_pkey; Type: CONSTRAINT; Schema: public; Owner: db_user
--

ALTER TABLE ONLY public."DetectedObject"
    ADD CONSTRAINT "DetectedObject_pkey" PRIMARY KEY ("ID");


--
-- Name: Model Model_pkey; Type: CONSTRAINT; Schema: public; Owner: db_user
--

ALTER TABLE ONLY public."Model"
    ADD CONSTRAINT "Model_pkey" PRIMARY KEY ("ID");


--
-- Name: Frame Frame_pkey; Type: CONSTRAINT; Schema: public; Owner: db_user
--

ALTER TABLE ONLY public."Frame"
    ADD CONSTRAINT "Frame_pkey" PRIMARY KEY ("ID");


--
-- Name: SensorData SensorData_pkey; Type: CONSTRAINT; Schema: public; Owner: db_user
--

ALTER TABLE ONLY public."SensorData"
    ADD CONSTRAINT "SensorData_pkey" PRIMARY KEY ("ID");


--
-- Name: DetectedObject DetectedObject_ModelID_fkey; Type: FK CONSTRAINT; Schema: public; Owner: db_user
--

ALTER TABLE ONLY public."DetectedObject"
    ADD CONSTRAINT "DetectedObject_ModelID_fkey" FOREIGN KEY ("ModelID") REFERENCES public."Model"("ID");


--
-- Name: SensorData SensorData_FrameID_fkey; Type: FK CONSTRAINT; Schema: public; Owner: db_user
--

ALTER TABLE ONLY public."SensorData"
    ADD CONSTRAINT "SensorData_FrameID_fkey" FOREIGN KEY ("FrameID") REFERENCES public."Frame"("ID");


--
-- Name: DetectedObject FrameID_fkey; Type: FK CONSTRAINT; Schema: public; Owner: db_user
--

ALTER TABLE ONLY public."DetectedObject"
    ADD CONSTRAINT FrameID_fkey FOREIGN KEY ("FrameID") REFERENCES public."Frame"("ID") NOT VALID;


--
-- PostgreSQL database dump complete
--

