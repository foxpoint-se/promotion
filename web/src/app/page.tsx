export default function Home() {
  return (
    <main className="flex min-h-screen flex-col items-center">
      <div className="flex min-h-screen flex-col items-center w-full pulsating-gradient p-2">
        <img
          src="/foxpoint_logo_outline.svg"
          className="md:max-h-64 max-h-32 md:mt-48 mt-24"
          alt="Foxpoint logo"
        />
        <p className="mt-14 md:text-3xl text-lg text-gray-700 text-center">
          Autonomous underwater vehicles.
        </p>
      </div>
    </main>
  );
}
