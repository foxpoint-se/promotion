// NOTE: I'm sorry for this :`(
const getPageButtonNumbers = (
  totalItems: number,
  pageSize: number,
  maxButtons: number,
  currentPage: number
): number[] => {
  const numberOfPages = Math.ceil(totalItems / pageSize);
  const lastPage = numberOfPages;
  const offsetUpAndDown = Math.floor(maxButtons / 2);
  const allButtons = Array.from({ length: numberOfPages }, (_, i) => i + 1);

  if (numberOfPages < maxButtons) {
    return allButtons;
  }

  if (
    currentPage + offsetUpAndDown <= lastPage &&
    currentPage - offsetUpAndDown > 0
  ) {
    return allButtons.slice(
      currentPage - offsetUpAndDown - 1,
      currentPage + offsetUpAndDown
    );
  }

  if (currentPage - offsetUpAndDown < 1) {
    const lowerBound = 0;
    if (allButtons.length > lastPage) {
      return allButtons.slice(lowerBound, lastPage);
    }
    return allButtons.slice(lowerBound, lowerBound + maxButtons);
  }

  return allButtons.slice(numberOfPages - maxButtons, numberOfPages);
};

export const Pagination = ({
  prevUrl,
  nextUrl,
  pageSize,
  totalItems,
  currentPage,
  basePath,
}: {
  prevUrl?: string;
  nextUrl?: string;
  pageSize: number;
  totalItems: number;
  currentPage: number;
  basePath: string;
}) => {
  const pageButtonNumbers = getPageButtonNumbers(
    totalItems,
    pageSize,
    5,
    currentPage
  );
  return (
    <div className="flex items-center">
      <a
        href={prevUrl}
        className={`btn btn-outline btn-sm mr-md ${
          !prevUrl ? "btn-disabled opacity-40" : ""
        }`}
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          fill="none"
          viewBox="0 0 24 24"
          strokeWidth={1.5}
          stroke="currentColor"
          className="w-5"
        >
          <path
            strokeLinecap="round"
            strokeLinejoin="round"
            d="M15.75 19.5L8.25 12l7.5-7.5"
          />
        </svg>
      </a>
      <div className="join join-horizontal">
        {pageButtonNumbers.map((n) => {
          let link: string | undefined = `${basePath}${n}`;
          if (n === currentPage) {
            link = undefined;
          } else if (n === 1) {
            link = basePath;
          }
          return (
            <a
              key={n}
              href={link}
              className={`btn btn-ghost btn-sm join-item 
            ${
              n === currentPage
                ? "btn-disabled !text-neutral-900 !bg-transparent !border-b-2 !border-b-primary"
                : ""
            }
            `}
            >
              {n}
            </a>
          );
        })}
      </div>
      <a
        href={nextUrl}
        className={`btn btn-outline btn-sm ml-md ${
          !nextUrl ? "btn-disabled opacity-40" : ""
        }`}
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          fill="none"
          viewBox="0 0 24 24"
          strokeWidth={1.5}
          stroke="currentColor"
          className="w-5"
        >
          <path
            strokeLinecap="round"
            strokeLinejoin="round"
            d="M8.25 4.5l7.5 7.5-7.5 7.5"
          />
        </svg>
      </a>
    </div>
  );
};
