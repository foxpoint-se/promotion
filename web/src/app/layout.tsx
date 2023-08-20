import "./globals.css";
import { Secular_One } from "next/font/google";

const secularOne = Secular_One({ weight: ["400"], subsets: ["latin"] });

export const metadata = {
  title: "Foxpoint",
  description: "Autonomous underwater vehicles",
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <html lang="en">
      <body className={`${secularOne.className}`}>{children}</body>
    </html>
  );
}
