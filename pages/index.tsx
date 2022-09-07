import type { NextPage } from "next";
import Head from "next/head";
import styles from "../styles/Home.module.css";

const Home: NextPage = () => {
  return (
    <div className={styles.container}>
      <Head>
        <title>Foxpoint</title>
        <meta name="description" content="Foxpoint" />
        <link rel="icon" href="/favicon.ico" />
      </Head>

      <main className={styles.main}>
        <div style={{ height: 200 }}>
          <img
            style={{ height: "100%", width: "auto" }}
            src="/foxpoint_logo_outline.svg"
          />
        </div>
      </main>
    </div>
  );
};

export default Home;
