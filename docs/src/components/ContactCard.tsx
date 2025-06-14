export const ContactCard = ({
  name,
  imgUrl,
  description,
  linkedin,
}: {
  name: string;
  imgUrl: string;
  description: string;
  linkedin: string;
}) => {
  return (
    <div className="card card-compact bg-transparent border border-neutral-300 shadow-lg w-full">
      <div className="card-body">
        <div className="flex items-start gap-md">
          <div className="shrink-0">
            <img className="h-44" src={imgUrl} />
          </div>
          <div className="flex flex-col">
            <div className="mb-2xl">
              <h2 className="font-semibold text-left text-3xl mb-sm">{name}</h2>
              <p className="text-left text-neutral-500">{description}</p>
            </div>
            <div className="text-lg text-left flex items-center gap-sm">
              <svg
                width="24"
                height="24"
                viewBox="0 0 65 65"
                xmlns="http://www.w3.org/2000/svg"
                className="fill-current"
              >
                <path d="M55.204 55.2039H45.604V40.1699C45.604 36.5849 45.54 31.9699 40.611 31.9699C35.611 31.9699 34.846 35.8759 34.846 39.9089V55.2029H25.246V24.2869H34.462V28.5119H34.591C35.5133 26.935 36.8461 25.6377 38.4473 24.7583C40.0486 23.8788 41.8584 23.4502 43.684 23.5179C53.414 23.5179 55.208 29.9179 55.208 38.2439L55.204 55.2039ZM14.414 20.0609C13.3122 20.0611 12.235 19.7346 11.3187 19.1226C10.4025 18.5106 9.68832 17.6407 9.26648 16.6228C8.84464 15.6049 8.73409 14.4848 8.94885 13.4041C9.16362 12.3234 9.69404 11.3306 10.473 10.5513C11.252 9.77208 12.2446 9.24132 13.3252 9.02617C14.4058 8.81101 15.526 8.92114 16.544 9.34261C17.562 9.76408 18.4322 10.478 19.0446 11.394C19.6569 12.31 19.9838 13.3871 19.984 14.4889C19.9841 15.2205 19.8402 15.945 19.5603 16.6209C19.2805 17.2969 18.8702 17.9111 18.353 18.4285C17.8358 18.9459 17.2217 19.3564 16.5458 19.6365C15.87 19.9166 15.1456 20.0608 14.414 20.0609ZM19.214 55.2039H9.604V24.2869H19.214V55.2039ZM59.99 0.00392588H4.78003C3.52692 -0.0102155 2.3194 0.473721 1.42291 1.34939C0.526428 2.22506 0.014317 3.42083 -0.000976562 4.67392V60.1129C0.0137935 61.3666 0.525604 62.5632 1.42206 63.4398C2.31851 64.3164 3.5263 64.8013 4.78003 64.7879H59.99C61.2462 64.8037 62.4574 64.3202 63.3574 63.4436C64.2574 62.5671 64.7726 61.3691 64.79 60.1129V4.66993C64.7721 3.41434 64.2565 2.21722 63.3564 1.34157C62.4564 0.465918 61.2456 -0.0166353 59.99 -7.57339e-05"></path>
              </svg>
              <a
                href={linkedin}
                target="_blank"
                aria-label={`${name} at LinkedIn`}
                className="link link-hover inline-block"
              >
                @{name}
              </a>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};
